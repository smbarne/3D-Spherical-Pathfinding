// node.h
// A simple class that defintes a position on a map.  A node may contain weights as well as terraintype / nodetype
// properties.
// Author: Stephen Barnes
// Created: 2011

#ifndef HEADER_NODE_INCLUDED
#define HEADER_NODE_INCLUDED
#if (MSC_VER > 1000)
	#pragma once
#endif

#include "stdafx.h"

class Vector2D;

class Node
{
public:
	// Basic terrain types - currently land represent passable terrain while water
	// represents unpassable terrain.
	enum TerrainType
	{
		LAND = 0,
		WATER
	};

	// Special terrain node type cases.  Can be later expanded to powerup, boost, etc.
	enum NodeType
	{
		BASIC = 0,
		START,
		FINISH
	};

	// Needed for STL Heap to compare class values when sorting
	// Compare using the f(n) value
	class NodeCompare 
	{
	public:
		bool operator() ( const Node *x, const Node *y ) const
		{
			return x->_f > y->_f;
		}
	};

	// Needed for checking if a Node has already been added to a heap.
	// Compare using the g(n) and h(n) individually
	class DetailedNodeCompare 
	{
	public:
		const Node* to_find;
		bool operator() ( const Node *other ) const
		{
			return ( (to_find->_pos == other->_pos) &&
			 (to_find->_f > other->_f));
		}
	};

	Node() {_g = 0.0f;	_h = 0.0f;	_f = 0.0f;};

	/** 
    * @param pos - Map position
    * @param mov - Ship movement stored at this position
	* @param tt - Terrain Type Attribute
	* @param nt - Node Type Attribute
    */
	Node(Vector2D *pos, Vector2D *mov, TerrainType tt, NodeType nt);
	~Node();

	// Utility Methods
	Node* Copy();
	bool CheckForWater();

	// Get Methods
	Vector2D*	GetPosition() { return _pos; };
	Vector2D*	GetMovementVec() { return _mov; };
	TerrainType GetTerrainType() { return _tt; };
	NodeType	GetNodeType() { return _nt; };
	Node*		GetChild() { return _child; };
	Node*		GetParent() { return _parent; };
	float		GetG() { return _g; };
	float		GetH() { return _h; };
	float		GetF() { return _f; };
	
	// Set Methods
	void SetPosition(Vector2D *pos) { _pos = pos; };
	void SetMovementVec(Vector2D *mov) { _mov = mov; };
	void SetTerrainType(TerrainType tt) { _tt = tt; };
	void SetNodeType(NodeType nt) { _nt = nt; };
	void SetChild(Node* child) { _child = child; };
	void SetParent(Node* parent) { _parent = parent; };
	void setG(float g) { _g = g; };
	void setH(float h) { _h = h; };
	void setF(float f) { _f = f; };

	bool operator()(const Node &other) const
    {
		if ( (_pos == other._pos) &&
			 (_g == other._g) &&
			 (_h == other._h))
				return true;
		return false;
    }

	bool operator==(const Node &other) const
    {
		if ( (_pos == other._pos) &&
			 (_g == other._g) &&
			 (_h == other._h))
				return true;
		return false;
    }

private:
	Vector2D *_pos;
	Vector2D *_mov;
	TerrainType _tt;
	NodeType _nt;

	Node* _parent;
	Node* _child;

	float _g; // The cost to get to this node
	float _h; // The heuristic value from h(n)
	float _f; // The sum of g(n)+h(n)
};
#endif //HEADER_NODE_INCLUDED