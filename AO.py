import heapq


class Node:
    def __init__(self, x, y, cost, heuristic):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        
    def __lt__(self, other):
        return (self.cost + self.heuristic) > (other.cost + other.heuristic)

dx = [-1, 0, 1, 0]
dy = [0, 1, 0, -1]


def heuristic(x, y, goalX, goalY):
    return abs(x - goalX) + abs(y - goalY)


def aStar(grid, startX, startY, goalX, goalY):
    
    pq = []
    heapq.heappush(pq, Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY)))


    visited = {}

    
    path = []

    
    while pq:
        
        node = heapq.heappop(pq)

        
        if node.x == goalX and node.y == goalY:
            while node.x != startX or node.y != startY:
                path.append((node.x, node.y))
                node.cost -= heuristic(node.x, node.y, goalX, goalY)
                for i in range(4):
                    x = node.x + dx[i]
                    y = node.y + dy[i]
                    if grid[x][y] == node.cost:
                        node.x = x
                        node.y = y
                        break
            path.append((startX, startY))
            path.reverse()
            return path

        
        visited[(node.x, node.y)] = True

        
        for i in range(4):
            x = node.x + dx[i]
            y = node.y + dy[i]
            if x >= 0 and x < len(grid) and y >= 0 and y < len(grid[0]) and (x, y) not in visited:
                heapq.heappush(pq, Node(x, y, node.cost + grid[x][y], heuristic(x, y, goalX, goalY)))

    
    return path

