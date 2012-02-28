// search.h
// A multi-approach, multithreaded spherical search implementation
// Author: Stephen Barnes
// Created: 2011

#ifndef HEADER_SEARCH_INCLUDED
#define HEADER_SEARCH_INCLUDED
#if (MSC_VER > 1000)
	#pragma once
#endif

#include "stdafx.h"
#include "node.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <osg/Vec2>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

class Node;

/// Supported Search Algorithm Types
enum SEARCH_TYPE{ 
	SEARCH_ASTAR = 0,
	SEARCH_BEAM,
	SEARCH_LOCAL,
	SEARCH_NUMVALUES
};

/// A custom 2D Vector class used to override operators, making specific game logic and search comparisons easier.
class Vector2D
{
public:
	
	/**  
    * @param x
    * @param y
    */
	Vector2D(int X, int Y) { x = X; y = Y;};
	Vector2D()  { x = 0; y = 0;};

	// Data	
	int x;
	int y;

	// Set Methods
	void SetXY(int X, int Y) { x = X; y = Y;};

	// Utility Methods
	/// Calculate linear distance between two points
	float CalcLinearDistanceF(int X, int Y);

	/// Calculate the spherical distance between two points in U,V space given a map of height and width
	float CalcSphericalDistanceF(int X, int Y, int height, int width);

	/// Calculate the length of the vector
	float LengthF();

	/// Get the maximum, integer, value of x and y
	int GetMax() { return std::max(abs(x),abs(y)); };

	// Overloaded operators
	Vector2D & operator+=(const Vector2D &rhs) {
		this->x = this->x + rhs.x;
		this->y = this->y + rhs.y;
		return *this;
	}

	Vector2D & operator-=(const Vector2D &rhs) {
		this->x = this->x - rhs.x;
		this->y = this->y - rhs.y;
		return *this;
	}

	const Vector2D operator+(const Vector2D &other) const {
		Vector2D result = *this;
		result += other;
		return result;
	}

	const Vector2D operator-(const Vector2D &other) const {
		Vector2D result = *this;
		result -= other;
		return result;
	}

	const Vector2D operator/(const int other) const {
		Vector2D result = *this;
		result.x = result.x / other;
		result.y = result.y / other;
		return result;
	}

	const bool operator()(const Vector2D &other) const {
		return (this->x == other.x) && (this->y == other.y);
	}

	const bool operator==(const Vector2D &other) const {
		return (this->x == other.x) && (this->y == other.y);
	}
};


/// The primary search class which contains the navmesh loaded into memory.
class Search
{
public:
	Search(std::string map) { Init(map); };
	
	mutable Node **mMapNodes;							// Global Land/Water 2D array of terrain nodes
	mutable std::vector<Node*> mFringeNodes;			// Vector/Heap of nodes on the fringe
	mutable std::vector<Node*> mExpandedNodes;			// Vector/Heap of expanded nodes 
	mutable Vector2D mStartPosition;					// Start position for each search iteration
	mutable Vector2D mFinishPosition;					// Destination position for each search iteration
	mutable OpenThreads::Mutex searchLock;				// Lock to ensure that map space memory is preserved per run

	bool Init( std::string map );

	// Run a search using the currently selected search algorithm.  Return a list of positions
	// so that an animation path can be created.
	std::vector<osg::Vec2f> Run(float startU, float startV, float endU, float endV);

	// Switch between various types of search algorithms
	void ToggleSearchType();

	// Get Methods
	int GetSearchType() { return meSearchType; };
	int GetExpandedNodeCount() { return (int)mExpandedNodes.size(); };
	bool GetSuccessfulRun() { return mbSuccessfulRun; };
	
protected:

	// Used to process the text map definition file.
	int GetMapHeight(std::string inputString);
	int GetMapWidth(std::string inputString);

	// Check to see if a terrain node is viable
	bool CheckForWater(int i, int j);

	// Primary search calls
	Node* PerformAStarSearch();
	Node* PerformLimitedMemoryAStar();
	Node* PerformLocalBeamSearch();
	
	std::vector<Node*> CalculateMoveOptions(Node* parent);
	std::vector<osg::Vec2f> ProcessSearchResults(Node* finalNode);
	
	// Data
	int			mHeight;		// Map Height
	int			mWidth;			// Map Width
	SEARCH_TYPE meSearchType;	// Current specified search algorithm
	bool		mbVerbose;		// Print debugging data
	int			mK;				// Maximum branching factor
	int			mMaxDelta;		// Maximum change in position per tick.  Effective acceleration. 
	bool		mbSuccessfulRun;// Successfully navigated path
	
};

#endif //HEADER_SEARCH_INCLUDED
