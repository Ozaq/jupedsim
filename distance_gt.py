#! /usr/bin/env python

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import skfmm
import rasterio.features as rsf
import shapely as shp
import argparse
import pathlib
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon
from matplotlib.colors import ListedColormap


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", type=pathlib.Path)
    return parser.parse_args()


def plot_polygon(axes, geo, **kwargs):
    patches = []
    for poly in geo.geoms:
        path = Path.make_compound_path(
            Path(np.asarray(poly.exterior.coords)[:, :2]),
            *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors],
        )

        patches.append(PathPatch(path, **kwargs))
    collection = PatchCollection(
        patches, edgecolor="red", facecolor="none", hatch="x"
    )

    axes.add_collection(collection)
    return axes


def fixup_geometry(geo):
    """
    Prepares the shapely polygons to be rasterized by rasterio.
    Polygons are scaled up to give us 5x5 pixel per square meter of geometry.
    The coordinates are shifted so that all coordinates are in the +x/+y quadrant.
    """
    geo = shp.affinity.scale(geo, 5, 5)
    geo = shp.affinity.translate(geo, -geo.bounds[0], -geo.bounds[1])
    return geo


def plot_rastermap_with_geometry_overlay(img, geo):
    figure = plt.figure()
    axes = figure.add_subplot(111)
    # plot_polygon(axes, geo)
    axes.set_xticks(np.arange(0, img.shape[1], 1), minor=True)
    axes.set_yticks(np.arange(0, img.shape[0], 1), minor=True)
    axes.locator_params(integer=True)
    # axes.grid(visible=True, which="both", color="silver", zorder=5)
    axes.imshow(
        img,
        zorder=0,
        extent=(0, img.shape[1], 0, img.shape[0]),
        origin="lower",
        interpolation="none",
        cmap=ListedColormap(["gainsboro", "lightyellow"]),
    )
    return axes
    # figure.savefig("overlay.png", dpi=254)


def main() -> int:
    args = parse_args()
    p = shp.Polygon([(0, 0), (3, 0), (3, 3), (0, 3)])
    geo = shp.GeometryCollection(
        [
            p,
            shp.affinity.translate(
                p,
                3.4,
                3.4,
            ),
            shp.affinity.translate(
                p,
                6.5,
                6.5,
            ),
            shp.affinity.translate(
                p,
                9.6,
                9.6,
            ),
            shp.affinity.translate(
                shp.affinity.scale(p, 0.75, 0.75),
                12,
                12,
            ),
        ]
    )
    geo = fixup_geometry(shp.from_wkt(args.filename.read_text()))

    px_width = int(np.ceil(geo.bounds[2] - geo.bounds[0]))
    px_height = int(np.ceil(geo.bounds[3] - geo.bounds[1]))
    img = rsf.rasterize(geo.geoms, (px_height, px_width))
    axes = plot_rastermap_with_geometry_overlay(img, geo)

    grid_size = img.shape
    grid_spacing = np.ones(grid_size.shape)
    boundary_indices = np.array([[1082 // 2, 6441 // 2]])
    boundary_times = np.array([0.0])
    uniform_speed = 1.0

    arrival_times = fmm.uniform_speed_signed_arrival_time(
        grid_size, boundary_indices, boundary_times, grid_spacing, uniform_speed
    )

    print("Max dist:", np.max(arrival_times[:]))

    plt.imshow(arrival_times)
    plt.plot(boundary_indices[0, 1], boundary_indices[0, 0], "mo")
    plt.colorbar()
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
