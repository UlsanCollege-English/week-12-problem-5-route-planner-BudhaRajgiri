import heapq
from collections import deque

def route_planner(graph, start, goal, weighted=True):
    """
    Compute the shortest path between start and goal.
    - If weighted=True: use Dijkstra's algorithm (graph edges are (neighbor, weight)).
    - If weighted=False: use BFS (graph edges are just neighbor names).
    Returns (path, cost). If unreachable, returns ([], None).
    """
    if start not in graph or goal not in graph:
        return ([], None)

    if start == goal:
        return ([start], 0)

    # --- Unweighted case: BFS ---
    if not weighted:
        visited = {start}
        parent = {start: None}
        queue = deque([start])

        while queue:
            node = queue.popleft()
            if node == goal:
                break
            for neighbor in graph.get(node, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    parent[neighbor] = node
                    queue.append(neighbor)

        if goal not in parent:
            return ([], None)

        # Reconstruct path
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parent.get(node)
        path.reverse()

        return (path, len(path) - 1)

    # --- Weighted case: Dijkstra ---
    dist = {node: float("inf") for node in graph}
    dist[start] = 0
    parent = {start: None}
    pq = [(0, start)]

    while pq:
        cost, node = heapq.heappop(pq)
        if cost > dist[node]:
            continue
        if node == goal:
            break
        for neighbor, weight in graph.get(node, []):
            new_cost = cost + weight
            if new_cost < dist[neighbor]:
                dist[neighbor] = new_cost
                parent[neighbor] = node
                heapq.heappush(pq, (new_cost, neighbor))

    if dist[goal] == float("inf"):
        return ([], None)

    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent.get(node)
    path.reverse()

    return (path, dist[goal])