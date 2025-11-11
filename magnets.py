import csv
import numpy as np
from math import pi

# Physical constant
MU0 = 4 * pi * 1e-7   # vacuum permeability

class Magnet:
    """
    Axis-magnetized ring magnet modelled with surface magnetic charges.
    You pass in Bz (measured at center of one pole face, on-axis), we estimate M = Bz / MU0.
    Geometry defaults are your values, in meters.
    """
    def __init__(self, Bz,
                 Ro=0.019202 / 2,
                 Ri=0.0062 / 2,
                 h=0.006424,
                 n_radial=10,
                 n_theta=60):
        self.Ro = Ro
        self.Ri = Ri
        self.h = h

        # Approximate uniform magnetization (A/m)
        # If Bz is pole-center field, this is an approximation, but good enough for your sim.
        self.M = 3.721557063559913 * (Bz * 0.001) / MU0

        self.n_radial = n_radial
        self.n_theta = n_theta

        # Precompute local patch positions + charges (in magnet's local frame)
        self.local_pos, self.q = self._make_patches()

    def _make_patches(self):
        """
        Build discrete patches on the two flat faces.
        Local frame:
            - z-axis = magnetization axis
            - faces at z = +/- h/2
        Top face = +M surface charge, bottom face = -M.
        """
        rs = np.linspace(self.Ri, self.Ro, self.n_radial + 1)
        thetas = np.linspace(0.0, 2.0 * pi, self.n_theta, endpoint=False)

        positions = []
        charges = []

        sigma = self.M  # surface pole density magnitude
        dtheta = 2.0 * pi / self.n_theta

        # face_sign = +1 for "north" face, -1 for "south" face in local coords
        for face_sign, z in ((+1.0, +self.h / 2.0),
                             (-1.0, -self.h / 2.0)):
            for i in range(self.n_radial):
                r_in = rs[i]
                r_out = rs[i + 1]
                r_mid = 0.5 * (r_in + r_out)
                dr = r_out - r_in
                dA = r_mid * dr * dtheta  # annular sector area per patch

                for theta in thetas:
                    x = r_mid * np.cos(theta)
                    y = r_mid * np.sin(theta)

                    positions.append([x, y, z])
                    charges.append(face_sign * sigma * dA)

        return np.array(positions, dtype=float), np.array(charges, dtype=float)

    def world_patches(self, center, R):
        """
        Transform local patch positions into world coordinates.

        center: (3,) array-like, magnet center in world coords.
        R: (3,3) rotation matrix from local -> world.
           For a magnet aligned with +z in world: R = np.eye(3).
        """
        center = np.asarray(center, dtype=float)
        R = np.asarray(R, dtype=float)
        world_pos = (R @ self.local_pos.T).T + center
        return world_pos, self.q


def force_between(mag1, center1, R1,
                  mag2, center2, R2):
    """
    Compute interaction force between two magnets using Coulomb-like
    interactions between their surface "magnetic charges".

    Returns:
        F_on_1, F_on_2  (each np.array shape (3,))
    """
    p1, q1 = mag1.world_patches(center1, R1)
    p2, q2 = mag2.world_patches(center2, R2)

    # Pairwise vectors: shape (N1, N2, 3)
    r_vec = p2[None, :, :] - p1[:, None, :]
    r2 = np.sum(r_vec**2, axis=2)

    # Avoid singularities
    mask = r2 > 1e-16
    if not np.any(mask):
        return np.zeros(3), np.zeros(3)

    r_vec = r_vec[mask]
    r2 = r2[mask]
    r = np.sqrt(r2)

    # Pairwise charge products in same masked order
    q_prod_full = q1[:, None] * q2[None, :]
    q_prod = q_prod_full[mask]

    # Coulomb-like magnetic force: (mu0 / 4pi) * (q1 q2 / r^3) * r_vec
    factors = (MU0 / (4.0 * pi)) * (q_prod / (r2 * r))  # = q1 q2 / r^3

    F_on_2 = np.sum(factors[:, None] * r_vec, axis=0)
    F_on_1 = -F_on_2

    return F_on_1, F_on_2


def load_magnets_from_csv(path,
                          Ro=0.019202 / 2,
                          Ri=0.0062 / 2,
                          h=0.006424,
                          n_radial=10,
                          n_theta=60):
    """
    Load magnets from a CSV.

    Expected CSV columns (example):
        magnet_id, Bz

    Returns:
        magnets: list[Magnet] in file order.
    """
    magnets = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            Bz = float(row['Bz'])
            m = Magnet(Bz,
                       Ro=Ro,
                       Ri=Ri,
                       h=h,
                       n_radial=n_radial,
                       n_theta=n_theta)
            magnets.append(m)
    return magnets

# Load your 5 magnets from csv
magnets = load_magnets_from_csv('magnets.csv')

# Example: pick two of them
m1 = magnets[0]
m2 = magnets[1]

# Rotation matrices (here both aligned with world z)
R_identity = np.eye(3)

def run_sim(d):
    # Positions at some timestep:
    C1 = np.array([0.0, 0.0, 0.0])
    C2 = np.array([0.0, 0.0, d/100])  # 10 mm above, same axis

    F_on_1, F_on_2 = force_between(m1, C1, R_identity,
                                   m2, C2, R_identity)

    return abs(F_on_1[-1])

grams = [7.98, 11.28, 14.16, 18.25, 22.4, 25.09]
dist = [2.1, 1.8, 1.6, 1.4, 1.2, 1.1]

a, p = [], []
r = 0



