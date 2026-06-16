import math
from pathlib import Path

import yaml
from PIL import Image


MAP_YAML = Path("labyrinth.yaml")
GRAPH_PIXELS = Path("graph_pixels.yaml")
GRAPH_OUT = Path("graph.yaml")
IMAGE_PATH = Path("labyrinth.png")

ANCHOR_NODE = "1"
ANCHOR_MAP_X = 0.0
ANCHOR_MAP_Y = 0.0


def normalize_edges(edges):
    result = []

    if isinstance(edges, list):
        for edge in edges:
            a, b = edge
            result.append((str(a), str(b)))
        return result

    if isinstance(edges, dict):
        seen = set()

        for a, neighbors in edges.items():
            a = str(a)

            for b in neighbors:
                b = str(b)
                key = tuple(sorted((a, b)))

                if key not in seen:
                    seen.add(key)
                    result.append((a, b))

        return result

    raise ValueError("edges must be list or dict")


with MAP_YAML.open() as f:
    map_data = yaml.safe_load(f)

resolution = float(map_data["resolution"])

img = Image.open(IMAGE_PATH)
width, height = img.size

with GRAPH_PIXELS.open() as f:
    graph = yaml.safe_load(f)

nodes = {str(name): node for name, node in graph["nodes"].items()}
edges = normalize_edges(graph["edges"])

anchor_px, anchor_py = nodes[ANCHOR_NODE]["pixel"]

out_nodes = {}

for name, node in nodes.items():
    px, py = node["pixel"]

    x = ANCHOR_MAP_X + (px - anchor_px) * resolution
    y = ANCHOR_MAP_Y - (py - anchor_py) * resolution

    out_nodes[name] = {
        "pixel": [px, py],
        "x": round(x, 4),
        "y": round(y, 4),
        "theta": 0.0,
        "type": node.get("type", "intermediate"),
    }

adjacency = {name: [] for name in out_nodes}
edge_lengths = {}

for a, b in edges:
    if a not in out_nodes:
        raise ValueError(f"Unknown node in edge: {a}")
    if b not in out_nodes:
        raise ValueError(f"Unknown node in edge: {b}")

    adjacency[a].append(b)
    adjacency[b].append(a)

    ax = out_nodes[a]["x"]
    ay = out_nodes[a]["y"]
    bx = out_nodes[b]["x"]
    by = out_nodes[b]["y"]

    length = math.hypot(ax - bx, ay - by)
    edge_lengths[f"{a}-{b}"] = round(length, 4)

for name in adjacency:
    adjacency[name] = sorted(adjacency[name], key=int)

out = {
    "map": {
        "image": str(IMAGE_PATH),
        "resolution": resolution,
        "width": width,
        "height": height,
        "anchor_node": ANCHOR_NODE,
        "anchor_pixel": [anchor_px, anchor_py],
        "anchor_map": [ANCHOR_MAP_X, ANCHOR_MAP_Y],
        "note": "pixel x совпадает с map x, pixel y инвертирован относительно map y",
    },
    "nodes": dict(sorted(out_nodes.items(), key=lambda item: int(item[0]))),
    "edges": dict(sorted(adjacency.items(), key=lambda item: int(item[0]))),
    "edge_lengths": edge_lengths,
}

with GRAPH_OUT.open("w") as f:
    yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)

print(f"saved {GRAPH_OUT}")
print(f"image size: {width}x{height}")
print(f"resolution: {resolution}")
print()

for name, node in sorted(out_nodes.items(), key=lambda item: int(item[0])):
    print(
        f"{name}: "
        f"pixel={node['pixel']} "
        f"x={node['x']} "
        f"y={node['y']} "
        f"type={node['type']}"
    )