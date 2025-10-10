from collections import defaultdict
from collections import deque

print("abc")

graph = defaultdict(list)

f = open('graph1.txt')
for line in f:
    u, v = line.split()
    graph[u].append(v)
print(graph)


def bfs_path(graph, p, q):
    # initialize
    visited = {}  # to mark visited vertices
    queue = deque()  # a queue

    # add starting vertex and mark it as visited
    queue.append(p)
    visited[p] = True

    # main loop
    while len(queue) > 0:
        v = queue.popleft()  # take a vertex from the queue
        print(f'v: {v}\tQ: {queue}')

        # the destination vertex found
        if v == q:
            print("The path was found")
            return

        # otherwise continue traverse
        for u in graph[v]:  # for all it's neighbours...
            if u not in visited:  # ... not yet visited
                queue.append(u)  # add them to the queue and...
                visited[u] = True  # mark as visited

    # all paths traversed and the destination was not found
    print("No path found")
