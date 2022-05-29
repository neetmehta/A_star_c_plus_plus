#include <string>
#include <vector>
#include <iostream>
#include <queue>
#include <map>
#pragma once

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

class AStar
{
public:
	// Variables
	std::vector<std::string> map;
	Point start;
	Point goal;

	// Constructor
	AStar();
	AStar(std::vector<std::string> map) :map(map)
	{
		find_s_g();
	};

	// methods
	void find_s_g();
	void claculate_heuristic(Point& point);
	std::vector<Point> get_neighbors(Point point, std::vector<std::vector<bool>>& visited, std::map<std::vector<int>, std::vector<int>>& mp);
	void astar();
	void backtrack(std::map<std::vector<int>, std::vector<int>>& mp);
	void find_path();
	
	
	void visualize();
	
};

class Compare
{
public:
	bool operator()(const Point& a, const Point& b)
	{
		return a.heuristic > b.heuristic;
	}
};

