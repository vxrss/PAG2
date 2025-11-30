import arcpy
import math
import heapq
import os

arcpy.env.overwriteOutput = True

path = r"dane\L4_2_BDOT10k__OT_SKJZ_L.shp"
out_dir = r"\wyniki"

start_vertex = 1
end_vertex = 21022

TOLERANCJA = 0.5

SPEED_MAP = {
    "A": 140,
    "S": 120,
    "GP": 100,
    "G": 90,
    "Z": 50,
    "L": 50,
    "D": 50,
    "I": 50
}

MAX_SPEED_M_S = max(SPEED_MAP.values()) / 3.6


def load_data(path: str):
    fields = ["Shape@", "klasaDrogi"]
    data = []
    with arcpy.da.SearchCursor(path, fields) as cur:
        for geom, klasa in cur:
            if not geom or not geom.firstPoint or not geom.lastPoint:
                continue

            start = (geom.firstPoint.X, geom.firstPoint.Y)
            end = (geom.lastPoint.X, geom.lastPoint.Y)
            dlugosc = float(geom.length)

            data.append({
                "start": start,
                "end": end,
                "length": dlugosc,
                "klasa": klasa
            })
    return data


def find_or_create_vertex(point, vertices):
    for vid, (x, y) in vertices.items():
        if math.hypot(x - point[0], y - point[1]) <= TOLERANCJA:
            return vid
    new_id = len(vertices) + 1
    vertices[new_id] = point
    return new_id


data = load_data(path)
vertices = {}
edges = []

for edge_id, rec in enumerate(data, start=1):
    v_from = find_or_create_vertex(rec["start"], vertices)
    v_to = find_or_create_vertex(rec["end"], vertices)

    speed = SPEED_MAP.get(rec["klasa"], 50)
    speed_m_s = speed / 3.6
    travel_time = rec["length"] / speed_m_s

    edges.append([
        edge_id,
        v_from,
        v_to,
        rec["klasa"],
        rec["length"],
        travel_time
    ])

graph = {v: [] for v in vertices.keys()}

for e in edges:
    edge_id, v_from, v_to, klasa, dl, t = e
    graph[v_from].append((v_to, dl, t))
    graph[v_to].append((v_from, dl, t))


def dijkstra(graph, start, goal, use_time=False):
    d = {v: math.inf for v in graph}
    p = {v: None for v in graph}
    d[start] = 0

    visited = set()
    Q = [(0, start)]
    neighbors_checked = 0

    while Q:
        dist_v, v = heapq.heappop(Q)
        if v in visited:
            continue
        visited.add(v)

        if v == goal:
            break

        for u, dist, time in graph[v]:
            neighbors_checked += 1

            cost = time if use_time else dist
            new_d = d[v] + cost

            if new_d < d[u]:
                d[u] = new_d
                p[u] = v
                heapq.heappush(Q, (new_d, u))

    path = []
    v = goal
    if d[goal] < math.inf:
        while v is not None:
            path.append(v)
            v = p[v]
        path.reverse()

    return path, d[goal], len(visited), neighbors_checked


def heuristic(v, goal):
    x1, y1 = vertices[v]
    x2, y2 = vertices[goal]
    return math.hypot(x1 - x2, y1 - y2)


def heuristic_time(v, goal):
    dist = heuristic(v, goal)
    return dist / MAX_SPEED_M_S


def astar_shortest(graph, start, goal):
    d = {v: math.inf for v in graph}
    p = {v: None for v in graph}
    d[start] = 0

    visited = set()
    Q = [(0, start)]
    neighbors_checked = 0

    while Q:
        f_v, v = heapq.heappop(Q)
        if v in visited:
            continue
        visited.add(v)

        if v == goal:
            break

        for u, dist, _ in graph[v]:
            neighbors_checked += 1
            new_d = d[v] + dist

            if new_d < d[u]:
                d[u] = new_d
                p[u] = v
                f_u = new_d + heuristic(u, goal)
                heapq.heappush(Q, (f_u, u))

    path = []
    v = goal
    if d[goal] < math.inf:
        while v is not None:
            path.append(v)
            v = p[v]
        path.reverse()

    return path, d[goal], len(visited), neighbors_checked


def astar_fastest(graph, start, goal):
    d = {v: math.inf for v in graph}
    p = {v: None for v in graph}
    d[start] = 0

    visited = set()
    Q = [(0, start)]
    neighbors_checked = 0

    while Q:
        f_v, v = heapq.heappop(Q)
        if v in visited:
            continue

        visited.add(v)
        if v == goal:
            break

        for u, dist, time in graph[v]:
            neighbors_checked += 1
            new_d = d[v] + time

            if new_d < d[u]:
                d[u] = new_d
                p[u] = v
                f_u = new_d + heuristic_time(u, goal)
                heapq.heappush(Q, (f_u, u))

    path = []
    v = goal
    if d[goal] < math.inf:
        while v is not None:
            path.append(v)
            v = p[v]
        path.reverse()

    return path, d[goal], len(visited), neighbors_checked


def calculate_path_length(path):
    total = 0
    for i in range(len(path) - 1):
        v = path[i]
        u = path[i + 1]
        for nei, dl, t in graph[v]:
            if nei == u:
                total += dl
                break
    return total


print("\n===== Dijkstra – najkrótsza =====")
path_D, dist_D, vs_D, ns_D = dijkstra(graph, start_vertex, end_vertex)
if path_D:
    print(f"Trasa: {path_D}")
    print(f"{dist_D / 1000:.3f} km")
    print(f"W S: {vs_D}, sąsiadów: {ns_D}")
else:
    print("Brak trasy.")

print("\n===== A* – najkrótsza =====")
path_A, dist_A, vs_A, ns_A = astar_shortest(graph, start_vertex, end_vertex)
if path_A:
    print(f"Trasa: {path_A}")
    print(f"{dist_A / 1000:.3f} km")
    print(f"W S: {vs_A}, sąsiadów: {ns_A}")
else:
    print("Brak trasy.")

print("\n===== A* – najszybsza =====")
path_F, time_F, vs_F, ns_F = astar_fastest(graph, start_vertex, end_vertex)
if path_F:
    length_F = calculate_path_length(path_F)
    print(f"Trasa: {path_F}")
    print(f"Długość: {length_F / 1000:.3f} km")
    print(f"Czas: {time_F / 60:.2f} min, {time_F / 3600:.3f} h)")
    print(f"W S: {vs_F}, sąsiadów: {ns_F}")
else:
    print("Brak trasy.")

if not os.path.exists(out_dir):
    os.makedirs(out_dir)

if path_F:
    out_shp_fast = os.path.join(out_dir, "fastest_path.shp")

    arcpy.management.CreateFeatureclass(out_dir, "fastest_path.shp",
                                        "POLYLINE",
                                        spatial_reference=path)

    arcpy.management.AddField(out_shp_fast, "start_id", "LONG")
    arcpy.management.AddField(out_shp_fast, "end_id", "LONG")
    arcpy.management.AddField(out_shp_fast, "czas_s", "DOUBLE")
    arcpy.management.AddField(out_shp_fast, "dl_m", "DOUBLE")

    points = [arcpy.Point(*vertices[v]) for v in path_F]
    polyline = arcpy.Polyline(arcpy.Array(points))

    with arcpy.da.InsertCursor(out_shp_fast,
                               ["SHAPE@", "start_id", "end_id", "czas_s", "dl_m"]) as cur:
        cur.insertRow((polyline, start_vertex, end_vertex, time_F, length_F))

    print(f"\nPlik najszybszej trasy zapisany:\n{out_shp_fast}")
else:
    print("\nBrak najszybszej trasy do zapisania.")

if path_A:
    out_shp_short = os.path.join(out_dir, "shortest_path.shp")

    arcpy.management.CreateFeatureclass(out_dir, "shortest_path.shp",
                                        "POLYLINE",
                                        spatial_reference=path)

    arcpy.management.AddField(out_shp_short, "start_id", "LONG")
    arcpy.management.AddField(out_shp_short, "end_id", "LONG")
    arcpy.management.AddField(out_shp_short, "czas_s", "DOUBLE")
    arcpy.management.AddField(out_shp_short, "dl_m", "DOUBLE")

    points = [arcpy.Point(*vertices[v]) for v in path_A]
    polyline = arcpy.Polyline(arcpy.Array(points))

    total_time = 0
    for i in range(len(path_A) - 1):
        v = path_A[i]
        u = path_A[i + 1]
        for nei, dl, t in graph[v]:
            if nei == u:
                total_time += t
                break

    with arcpy.da.InsertCursor(out_shp_short,
                               ["SHAPE@", "start_id", "end_id", "czas_s", "dl_m"]) as cur:
        cur.insertRow((polyline, start_vertex, end_vertex, total_time, dist_A))

    print(f"\nPlik najkrótszej trasy zapisany:\n{out_shp_short}")
else:
    print("\nBrak najkrótszej trasy do zapisania.")