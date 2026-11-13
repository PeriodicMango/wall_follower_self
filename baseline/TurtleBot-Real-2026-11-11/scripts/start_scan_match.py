"""Match a stationary planar laser scan against the full saved trinary occupancy grid."""
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


@dataclass(frozen=True)
class MatchResult:
    accepted: bool
    x: float | None
    y: float | None
    yaw: float | None
    score: float
    margin: float
    coverage: float
    reason: str


class StartScanMatcher:
    """Grid coordinates refer to the laser origin. Reported poses refer to the robot base in map."""

    def __init__(self, map_yaml, home, laser_pose=(0., 0., 0.), start_tolerance_m=.25):
        path = Path(map_yaml)
        meta = yaml.safe_load(path.read_text())
        if meta.get("mode", "trinary") != "trinary":
            raise ValueError("A trinary occupancy map is required")
        self.res = float(meta["resolution"])
        self.origin = tuple(map(float, meta["origin"]))
        self.home = tuple(map(float, home[:2]))
        self.laser = tuple(map(float, laser_pose))
        self.tolerance = float(start_tolerance_m)
        if (len(self.origin) != 3 or len(self.home) != 2 or len(self.laser) != 3 or
                not all(map(math.isfinite, (*self.origin, *self.home, *self.laser, self.res, self.tolerance))) or
                min(self.res, self.tolerance) <= 0):
            raise ValueError("Invalid map geometry, home or laser extrinsics")
        with Image.open(path.parent / meta["image"]) as image:
            gray = np.flipud(np.asarray(image.convert("L"), dtype=np.float32)) / 255.
        occupancy = gray if meta.get("negate", 0) else 1. - gray
        self.grid = np.where(occupancy > meta["occupied_thresh"], 1,
                             np.where(occupancy < meta["free_thresh"], 0, -1)).astype(np.int8)
        self.height, self.width = self.grid.shape
        # At most 10 cm between global samples on this course map. Reject an oversized
        # search explicitly instead of silently discarding possible competing poses.
        self.step = max(1, round(.10 / self.res))
        rows, cols = np.nonzero(self.grid[::self.step, ::self.step] == 0)
        if not 1 <= len(rows) <= 5000:
            raise ValueError("Unsupported search size: need 1–5000 known-free coarse positions")
        self.coarse_xy = (np.column_stack((cols, rows)) * self.step + .5) * self.res
        self.coarse_rays = self._rays(self.coarse_xy, 120)

    def _cells(self, xy):
        index = np.floor(xy / self.res).astype(int)
        x, y = index[:, 0], index[:, 1]
        inside = (x >= 0) & (x < self.width) & (y >= 0) & (y < self.height)
        return np.where(inside, self.grid[y.clip(0, self.height - 1), x.clip(0, self.width - 1)], -1)

    def _rays(self, xy, directions):
        """Distance to first occupied cell, or NaN when the ray reaches unknown first.

        Cross grid boundaries in ray order (DDA). A wall along a measured free
        ray is therefore visible even when there is another wall at its endpoint.
        """
        angle = np.arange(directions) * math.tau / directions
        dx = np.tile(np.cos(angle), len(xy))
        dy = np.tile(np.sin(angle), len(xy))
        x, y = np.repeat(xy[:, 0], directions), np.repeat(xy[:, 1], directions)
        ix, iy = np.floor(x / self.res).astype(int), np.floor(y / self.res).astype(int)
        sx, sy = np.sign(dx).astype(int), np.sign(dy).astype(int)
        def crossings(coordinate, cell, step, direction):
            delta = np.abs((cell + (step > 0)) * self.res - coordinate)
            moving = np.abs(direction) > 1e-12
            first = np.divide(delta, np.abs(direction), out=np.full(len(x), np.inf), where=moving)
            every = np.divide(self.res, np.abs(direction), out=np.full(len(x), np.inf), where=moving)
            return first, every
        tx, hx = crossings(x, ix, sx, dx)
        ty, hy = crossings(y, iy, sy, dy)
        rays = np.full(len(x), np.nan, dtype=np.float32)
        active = np.flatnonzero(np.repeat(self._cells(xy) == 0, directions))
        while active.size:
            left, down = tx[active], ty[active]
            step_x, step_y = left <= down, down <= left
            ax, ay = active[step_x], active[step_y]
            ix[ax] += sx[ax]
            tx[ax] += hx[ax]
            iy[ay] += sy[ay]
            ty[ay] += hy[ay]
            xx, yy = ix[active], iy[active]
            inside = (xx>=0) & (xx<self.width) & (yy>=0) & (yy<self.height)
            cells = self.grid[yy.clip(0,self.height-1), xx.clip(0,self.width-1)]
            hit = inside & (cells==1)
            rays[active[hit]] = np.minimum(left[hit],down[hit])
            active = active[inside & (cells==0)]
        return rays.reshape(len(xy), directions)

    def _base_xy(self, sensor_xy, laser_angles):
        yaw = laser_angles-self.laser[2]
        lx, ly, _ = self.laser
        return sensor_xy - np.column_stack((lx*np.cos(yaw)-ly*np.sin(yaw), lx*np.sin(yaw)+ly*np.cos(yaw)))

    def _score(self, xy, rays, yaw_indices, angles, obs, coarse):
        # For each laser origin evaluate all requested headings against actual hit beams.
        n = rays.shape[1]
        headings = np.broadcast_to(yaw_indices, (len(xy), yaw_indices.shape[-1]))
        beam_index = np.rint(angles*n/math.tau).astype(int)
        lookup = (headings[:,:,None] + beam_index) % n
        simulated = rays[np.arange(len(xy))[:,None,None],lookup]
        error = obs - simulated
        known = np.isfinite(simulated)
        sigma = (.11 if coarse else .075) + .015 * obs
        error_units = np.where(known, np.abs(error) / sigma, 8.)
        agreement = np.exp(-.5 * np.minimum(8., error_units)**2)
        # An unexpectedly near return can be clutter; a beam travelling well
        # through a mapped wall is stronger evidence against this pose.
        through_wall = error > 2*sigma
        scores = np.mean(agreement - .18*through_wall,axis=2)
        base = self._base_xy(np.repeat(xy,headings.shape[1],axis=0), headings.flatten()*math.tau/n)
        scores[self._cells(base).reshape(scores.shape)!=0] = -1.
        return scores, np.mean(known,axis=2)

    @staticmethod
    def _beams(ranges, amin, inc, rmin, rmax):
        ranges = np.asarray(ranges,dtype=np.float64)
        if ranges.ndim!=1 or not np.all(np.isfinite([amin,inc,rmin,rmax])) or inc==0 or rmax<=rmin:
            return None
        angles = (amin + inc*np.arange(len(ranges))) % math.tau
        good = np.isfinite(ranges) & (ranges>=max(.001,rmin)) & (ranges<rmax)
        if not np.any(good):
            return None
        # One valid observed hit per 6 degrees. Retain its actual beam angle.
        angles, ranges = angles[good], ranges[good]
        bins = np.rint(angles*60/math.tau).astype(int)%60
        errors = np.abs(np.angle(np.exp(1j*(angles-bins*math.tau/60))))
        ordered = np.argsort(errors,kind="stable")
        _, first = np.unique(bins[ordered],return_index=True)
        choose = ordered[first]
        if len(choose)<24 or len(np.unique((angles[choose]*8/math.tau).astype(int)))<6:
            return None
        return angles[choose],ranges[choose]

    @staticmethod
    def _different(a, b, translation, rotation_deg):
        return (np.linalg.norm(a[:2]-b[:2]) >= translation or
                abs(math.remainder(a[2]-b[2],360.)) >= rotation_deg)

    def _seeds(self, score):
        best = np.max(score)
        seeds = []
        for idx in np.argsort(score,axis=None)[::-1]:
            pos, yaw = np.unravel_index(idx,score.shape)
            if score[pos,yaw] < best-.16 and len(seeds) >= 8:
                break
            candidate = np.array([*self.coarse_xy[pos],float(yaw*6)])
            if all(self._different(candidate,other,.145,15.) for other in seeds):
                seeds.append(candidate)
                if len(seeds)>48:
                    return None
        return np.array(seeds)

    def _refine(self, seeds, offset, yaw_offsets, angles, obs):
        offsets = np.array([(x,y) for x in (-offset,0.,offset) for y in (-offset,0.,offset)])
        xy = (seeds[:,None,:2]+offsets).reshape(-1,2)
        # Overlapping translation neighbourhoods use the same ray table.
        unique, inv = np.unique(xy.round(7),axis=0,return_inverse=True)
        rays = self._rays(unique,360)[inv]
        headings = np.repeat(seeds[:,2].round().astype(int),len(offsets))[:,None]+yaw_offsets
        scores,covers = self._score(xy,rays,headings,angles,obs,False)
        s = scores.reshape(len(seeds),-1)
        indices = s.argmax(axis=1)
        pos, rot = np.unravel_index(indices,(len(offsets),len(yaw_offsets)))
        rows = np.arange(len(seeds))*len(offsets)+pos
        return np.column_stack((xy[rows],headings[rows,rot]%360)),scores[rows,rot],covers[rows,rot]

    def match(self, ranges, angle_min, angle_increment, range_min, range_max):
        def reject(reason):
            return MatchResult(False,None,None,None,0.,0.,0.,reason)
        beams = self._beams(ranges,angle_min,angle_increment,range_min,range_max)
        if beams is None:
            return reject("insufficient valid laser coverage")
        angles,obs = beams
        # Bound temporary pose × angle × beam arrays even for a larger saved grid.
        headings = np.arange(0, 120, 2)
        global_scores = np.empty((len(self.coarse_xy), len(headings)))
        for first in range(0, len(self.coarse_xy), 192):
            batch = slice(first, first + 192)
            global_scores[batch], _ = self._score(
                self.coarse_xy[batch], self.coarse_rays[batch], headings, angles, obs, True)
        if np.max(global_scores) < .38:
            return reject("poor global scan fit")
        seeds = self._seeds(global_scores)
        if seeds is None:
            return reject("ambiguous: too many globally competitive poses")
        mid,_,_ = self._refine(seeds,self.step*self.res/2,np.arange(-6,7,2),angles,obs)
        poses,scores,covers = self._refine(mid,self.step*self.res/4,np.arange(-2,3),angles,obs)
        win = np.argmax(scores)
        pose = poses[win]
        yaw = math.radians(pose[2])
        base = self._base_xy(pose[None,:2],np.array([yaw]))[0]
        ox,oy,ot = self.origin
        x,y = ox+base[0]*math.cos(ot)-base[1]*math.sin(ot), oy+base[0]*math.sin(ot)+base[1]*math.cos(ot)
        others = [scores[i] for i,p in enumerate(poses) if self._different(p,pose,.20,20.)]
        score, coverage = float(scores[win]),float(covers[win])
        # Unexpanded coarse modes are below the candidate band. Never report
        # greater confidence than the search band itself can establish.
        margin = min(.16, score-max(others)) if others else .16
        reason = "matched"
        if coverage < .65 or score < .62:
            reason = "insufficient mapped walls or poor scan fit"
        elif margin < .065:
            reason = "ambiguous competing map pose"
        elif math.hypot(x-self.home[0],y-self.home[1]) > self.tolerance:
            reason = "best map pose is outside the saved start"
        elif math.hypot(x-self.home[0],y-self.home[1]) + self.res / 2 > self.tolerance:
            reason = "uncertain at saved start boundary"
        return MatchResult(reason=="matched",float(x),float(y),math.remainder(yaw-self.laser[2]+ot,math.tau),
                           score,float(margin),coverage,reason)
