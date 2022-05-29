// A_star.cpp : Defines the entry point for the application.
//

#include <string>
#include <vector>
#include <iostream>
#include <queue>
#include <map>

struct Point;
struct Point1Compare;
std::vector<Point> get_neighbors(std::vector<std::string>& map, Point point, Point start, Point goal, std::vector<std::vector<bool>>& visited, std::map<std::vector<int>, std::vector<int>>& mp);
void visualize(std::vector<std::string> map);
void claculate_heuristic(const Point& start, const Point& goal, Point& point);
void backtrack(std::vector<std::string>& map, Point goal, std::map<std::vector<int>, std::vector<int>>& mp);
std::vector<std::string> map = {                                  "####################",
								  "#                  #",
								  "#                  #",
								  "#     #####        #",
								  "# S        #       #",
								  "#          #       #",
								  "###############    #",
								  "#  #               #",
								  "#  #     #####     #",
								  "#   #            G #",
								  "#    #             #",
								  "####################", };

struct Point
{
public:
	int x;
	int y;
	int home_step = 0;
	int heuristic = INT_MAX;
	Point(int x_cord, int y_cord) : x(x_cord), y(y_cord) {};
	Point() :x(0), y(0) {};
};


class Compare
{
public:
	bool operator()(const Point& a, const Point& b)
	{
		return a.heuristic > b.heuristic;
	}
};

void visualize(std::vector<std::string> map)
{
	for (int i = 0; i < map.size(); i++)
	{
		std::cout << map[i] << std::endl;
	}
}


void astar(std::vector<std::string>& map, Point& start, Point& goal)
{
	bool is_possible = false;
	std::map<std::vector<int>, std::vector<int>> mp;
	mp[std::vector<int>{start.x, start.y}] = std::vector<int>{ -1,-1 };
	std::priority_queue<Point, std::vector<Point>, Compare> pq;
	pq.push(start);
	Point point = start;
	std::vector<Point> neighbors;
	std::vector<std::vector<bool>> visited(map.size(), std::vector<bool>(map[0].size(), false));
	while (!pq.empty())
	{
		point = pq.top();
		pq.pop();
		visited[point.x][point.y] = true;
		if (point.x == goal.x && point.y == goal.y)
		{
			is_possible = true;
			break;
		}
		neighbors = get_neighbors(map, point, start, goal, visited, mp);
		for (int i = 0; i < neighbors.size(); i++)
		{
			pq.push(neighbors[i]);
		}
	}
	if (!is_possible)
	{
		std::cout << "no path possible" << std::endl;
		return;
	}
	backtrack(map, goal, mp);
	map[start.x][start.y] = 'S';
	map[goal.x][goal.y] = 'G';

}

void backtrack(std::vector<std::string>& map, Point goal, std::map<std::vector<int>, std::vector<int>>& mp)
{
	int x = goal.x, y = goal.y;
	int a, b;
	while (x != -1 && y != -1)
	{
		map[x][y] = '.';
		a = mp[std::vector<int>{x, y}][0];
		b = mp[std::vector<int>{x, y}][1];
		x = a;
		y = b;
	}
}

std::vector<Point> get_neighbors(std::vector<std::string>& map, Point point, Point start, Point goal, std::vector<std::vector<bool>>& visited, std::map<std::vector<int>, std::vector<int>>& mp)
{
	int x = point.x, y = point.y;
	std::vector<std::vector<int>> nbs{ {1,0},{0,1},{-1,0},{0,-1} };
	std::vector<Point> neighbors;
	for (std::vector<int> nb : nbs)
	{

		if (x - nb[0] >= 0 && x - nb[0] < map.size() && y - nb[1] >= 0 && y - nb[1] < map[0].size() && !visited[x - nb[0]][y - nb[1]])
		{
			if (map[x - nb[0]][y - nb[1]] == '#') continue;
			Point neighbor{ x - nb[0] , y - nb[1] };
			neighbor.home_step = point.home_step + 1;
			claculate_heuristic(start, goal, neighbor);
			mp[std::vector<int>{neighbor.x, neighbor.y}] = std::vector<int>{ point.x,point.y };
			neighbors.push_back(neighbor);
		}
	}
	return neighbors;
}

void claculate_heuristic(const Point& start, const Point& goal, Point& point)
{
	point.heuristic = point.home_step + abs(point.x - goal.x) + abs(point.y - goal.y);
}

void find_s_g(Point& start, Point& goal, std::vector<std::string>& map)
{
	for (int i = 0; i < map.size(); i++)
	{
		for (int j = 0; j < map[0].size(); j++)
		{
			if (map[i][j] == 'S')
			{
				start.x = i;
				start.y = j;
			}

			if (map[i][j] == 'G')
			{
				goal.x = i;
				goal.y = j;
			}
		}
	}
}

int main()
{
	Point start{ 0,0 };
	Point goal{ 9,17 };
	find_s_g(start, goal, map);
	visualize(map);
	astar(map, start, goal);
	visualize(map);
	return 0;
}
