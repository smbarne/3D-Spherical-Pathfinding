// node.cpp
// A simple class that defintes a position on a map.  A node may contain weights as well as terraintype / nodetype
// properties.
// Author: Stephen Barnes
// Created: 2011

#include "node.h"

/// Create a map node
Node::Node(Vector2D *pos, Vector2D *mov, Node::TerrainType tt, Node::NodeType nt)
{
	_pos = pos;
	_mov = mov;
	_tt = tt;
	_nt = nt;

	_parent = NULL;
	_child = NULL;

	_g = 0.0f;
	_h = 0.0f;
	_f = 0.0f;
}

Node::~Node()
{
}

/// Make an exact, deep copy of a map node
Node* Node::Copy()
{
	Node* copy = new Node(_pos, _mov, _tt, _nt);
	copy->SetParent(_parent);
	copy->SetChild(_child);
	copy->setG(_g);
	copy->setH(_h);
	copy->setF(_f);

	return copy;
}

/// Return true if the terrain type is water
bool Node::CheckForWater()
{
	return (GetTerrainType() == Node::WATER);
}

