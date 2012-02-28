// search.cpp
// A multi-approach, multithreaded spherical search implementation
// Author: Stephen Barnes
// Created: 2011

// Contains:
// - Search class initialization
// - Implemenations for:
//		- A* Search
//		- Beam Search
//		- Local Beam Search
// - Game logic for determinining movement options
// - Utility methods for accessing search data

#include "search.h"
#include "node.h"

using namespace std;

#define DEBUG 0
#define UPPER_SEARCH_LIMIT 25000

/// Initialize the search class and load the navmesh/map into memory
bool Search::Init( std::string map )
{
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(searchLock);
	std::string strFilename = map;
	mStartPosition = Vector2D(0,0);	
	meSearchType = SEARCH_ASTAR;
	mbVerbose = false;
	mbSuccessfulRun = true;
	mK = 100;
	mMaxDelta = 1;
	
	// Verify input values are read in correctly
	if (mbVerbose)
	{
		std::cout << "Input filename: [" << strFilename << "]" << std::endl;
		std::cout << "Branching factor: [" << mK << "]" <<  std::endl;
	}

	// Begin loading map file into memory
	ifstream inputFile;
	std::string buffer;
	mWidth=0, mHeight=0;
	inputFile.open (strFilename.c_str());
	if (inputFile.is_open())
	{	
		if (inputFile.good())
		{
			std::getline(inputFile,buffer);
			mWidth = GetMapWidth(buffer);
			if (mbVerbose)
				std::cout << "Width is: [" << mWidth << "]" << std::endl;
		}
		if (inputFile.good())
		{
			std::getline(inputFile,buffer);
			mHeight = GetMapHeight(buffer);
			if (mbVerbose)
				std::cout << "Height is: [" << mHeight << "]" << std::endl;
		}

		//Create 2D array of width and height to store the map data
		mMapNodes = new Node*[mWidth];
		for (int i = 0; i < mWidth; ++i)
			mMapNodes [i] = new Node[mHeight+1];

		if (mbVerbose)
			std::cout << "Loading map into memory...";

		int j = 0;
		while ( inputFile.good() )
		{
			std::string buffer;
			std::getline(inputFile,buffer);
			if ((int)buffer.size() >= mWidth) // There is a newline at the end of each map file to watch out for
			{
				for(int i = 0; i < mWidth; i++)
				{	
					switch(buffer.at(i))
					{
						case ' ':						// ' ' = road
							mMapNodes[i][j] = Node(new Vector2D(i,j), new Vector2D(0,0),Node::LAND, Node::BASIC);
							break;
						case 'm': // 'm' = off road
						case '#': // '#' = off road
							mMapNodes[i][j] = Node(new Vector2D(i,j), new Vector2D(0,0),Node::WATER, Node::BASIC);
							break;
						case 'S':						// 'S' = Start (land)
							mMapNodes[i][j] = Node(new Vector2D(i,j), new Vector2D(0,0), Node::LAND, Node::START);
							mStartPosition = Vector2D(i,j);
							break;
						case 'F':						// 'F' - Finish (land)
							mMapNodes[i][j] = Node(new Vector2D(i,j), new Vector2D(0,0), Node::LAND, Node::FINISH);
							mFinishPosition = Vector2D(i,j);
							break;
						default:						// Error case
							std::cout << "Error: Unexpected character [" << buffer.at(i) << "] assuming off terrain (water)" << std::endl;
							mMapNodes[i][j] = Node(new Vector2D(i,j), new Vector2D(0,0), Node::WATER, Node::BASIC);
							break;
					}
				}
			}
			j++;

			if (mbVerbose)
				std::cout << ".";
		}
		inputFile.close();
		if (mbVerbose)
			std::cout << std::endl;

		// Verify input values are read in correctly
		if (mbVerbose)
		{
			if (0) // Don't reprint the map unless something really bad happens
			{
				for (int i=0; i<mWidth; i++)
				{
					for (int j=0; j<mHeight; j++)
					{
						if (mMapNodes[i][j].GetTerrainType() == Node::LAND)
							std::cout << " ";
						else
							std::cout << "#";
					}
					std::cout << std::endl;
				}
			}

			std::cout << "Start Position:" << std::endl;
			std::cout << mStartPosition.x << "," << mStartPosition.y << " -- ";

			std::cout << "\nFinish Position:" << std::endl;
			std::cout << mFinishPosition.x << "," << mFinishPosition.y << " -- ";
			std::cout << std::endl;
			
		}
		return true;
	}
	else
		cout << "Unable to open file"; 
	
	return false;
}

/// Run a search using the currently selected search algorithm.  Return a list of positions
/// so that an animation path can be created.
std::vector<osg::Vec2f> Search::Run(float startU, float startV, float endU, float endV)
{
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(searchLock);

	mMapNodes[mStartPosition.x][mStartPosition.y].SetNodeType(Node::BASIC);
	mMapNodes[mFinishPosition.x][mFinishPosition.y].SetNodeType(Node::BASIC);
	
	mStartPosition.SetXY(startU*mWidth, startV*mHeight);
	mFinishPosition.SetXY(endU*mWidth, endV*mHeight);
	
	if (mbVerbose)
	{
		std::cout << "Width: [" << mWidth << "] Height: [" << mHeight << "]" << std::endl;
		std::cout << "Start Position:" << std::endl;
		std::cout << mStartPosition.x << "," << mStartPosition.y << " -- ";
		
		std::cout << "\nFinish Position:" << std::endl;
		std::cout << mFinishPosition.x << "," << mFinishPosition.y << " -- ";
		std::cout << std::endl;
	}
	
	// Catch any unexpected values
	if (mStartPosition.x >= mWidth)
		mStartPosition.x = mWidth-1;
	else if(mStartPosition.x < 0)
		mStartPosition.x = 0;

	if (mStartPosition.y >= mHeight)
		mStartPosition.y = mHeight-1;
	else if (mStartPosition.y < 0)
		mStartPosition.y = 0;

	mMapNodes[mStartPosition.x][mStartPosition.y].SetNodeType(Node::START);
	mMapNodes[mFinishPosition.x][mFinishPosition.y].SetNodeType(Node::FINISH);

	// Check to make sure that the destination point is valid and on land
	if (mMapNodes[mFinishPosition.x][mFinishPosition.y].CheckForWater())
	{
		if (mbVerbose)
			std::cout << "Found water" << std::endl;
		std::vector<osg::Vec2f> none;
		return none;
	}
	else
		if (mbVerbose)
			std::cout << "Found land" << std::endl;

	// Check to see if our starting position is on water.  If so, we could already be stuck.
	if (mMapNodes[mStartPosition.x][mStartPosition.y].CheckForWater())
	{
		std::cout << "I'm stuck!" << std::endl;
		std::vector<osg::Vec2f> none;
		return none;
	}

	// Let's move along now...
	Node* finalNode = NULL;
	switch (meSearchType)
	{
		case SEARCH_ASTAR:
		default:
			finalNode = PerformAStarSearch();
			break;
		case SEARCH_BEAM:
			finalNode = PerformLimitedMemoryAStar();
			break;
		case SEARCH_LOCAL:
			finalNode = PerformLocalBeamSearch();
			break;
	}
	
	if (mbVerbose)
	{
		if (finalNode)
			std::cout << "finalNode pos: [" << finalNode->GetPosition()->x << "," << finalNode->GetPosition()->y << "]" << std::endl;
		else 
			std::cout << "Can't find a path!" << std::endl;
	}

	return ProcessSearchResults(finalNode);
}

/// Cycle through the available search algorithm types
void Search::ToggleSearchType()
{
	if (meSearchType < SEARCH_NUMVALUES-1)
		meSearchType = (SEARCH_TYPE)(meSearchType + 1);
	else
		meSearchType = (SEARCH_TYPE)0;
}

/// Parse the inputString for the width data
int Search::GetMapWidth(std::string inputString)
{
	size_t found;
	found = inputString.find_first_not_of("WIDTH ", 6);
	if (found!=string::npos)
	{
		return atoi(inputString.substr(found,inputString.length()).c_str());
	}
	return 0;
}

/// Parse the inputString for the height data
int Search::GetMapHeight(std::string inputString)
{
	size_t found;
	found = inputString.find_first_not_of("HEIGHT ", 7);
	if (found!=string::npos)
	{
		return atoi(inputString.substr(found,inputString.length()).c_str());
	}
	return 0;
}

/// Check a map position for water (inpassable terrain)
bool Search::CheckForWater(int i, int j)
{
	return (mMapNodes[i][j].GetTerrainType() == Node::WATER);
}

/// An A* Pathfinding Search Implementation.
Node* Search::PerformAStarSearch()
{
	// Reset from earlier search runs
	mFringeNodes.clear();
	mExpandedNodes.clear();
	mbSuccessfulRun = true;

	Node* currentNode = &mMapNodes[mStartPosition.x][mStartPosition.y];
	currentNode->setF(currentNode->GetH());
	std::vector<Node*> movementOptions;

	while(1)
	{
		movementOptions = CalculateMoveOptions(currentNode);

		// Add movement options to fringe heap, choose lowest cost from fringe for the next search node
		for (std::vector<Node*>::size_type i=0; i<movementOptions.size();i++)
		{			
			Node::DetailedNodeCompare finder;
			finder.to_find = movementOptions.at(i);

			if ( (find_if(mExpandedNodes.begin(), mExpandedNodes.end(), finder) == mExpandedNodes.end()) &&
				 (find_if(mFringeNodes.begin(), mFringeNodes.end(), finder) == mFringeNodes.end()))
			{
				mFringeNodes.push_back(movementOptions.at(i));
				push_heap( mFringeNodes.begin(), mFringeNodes.end(), Node::NodeCompare() ); // Sort back element into Heap
			}
		}

		// No way there, or, there seems to be a problem, exit out before wasting too much time
		if ( (mFringeNodes.size() == 0) || ((int)mExpandedNodes.size() > UPPER_SEARCH_LIMIT))
		{
			mbSuccessfulRun = false;
			break;
		}

		// Pop the best node (the one with the lowest f) 
		// Add the current node to the expanded list.  Remove from fringe 
		currentNode = mFringeNodes.front(); 
		// Keep track of expanded nodes for summary data
		mExpandedNodes.push_back(mFringeNodes.front());
		pop_heap( mFringeNodes.begin(), mFringeNodes.end(), Node::NodeCompare() );
		mFringeNodes.pop_back();

		// Then clear movementOptions
		movementOptions.clear();

		if (DEBUG && mbVerbose)
			std::cout << "CNode: (" << currentNode->GetPosition()->x << "," << currentNode->GetPosition()->y << ") g=[" << currentNode->GetG() << 
						"] h=[" << currentNode->GetH() << "] f=" << currentNode->GetF() << "] mov: (" << currentNode->GetMovementVec()->x << "," <<
						currentNode->GetMovementVec()->y << ")\n";

		// End case check
		if (*currentNode->GetPosition() == mFinishPosition)
			break;
	}

	if (mbVerbose)
		std::cout << "Nodes Expanded= " << mExpandedNodes.size() << std::endl;

	return currentNode;
}

/// Beam Search Implementation.
/// Keeps mK nodes in memory.
Node* Search::PerformLimitedMemoryAStar()
{
	// Reset from earlier search runs
	mFringeNodes.clear();
	mExpandedNodes.clear();
	mbSuccessfulRun = true;

	std::vector<Node*> currentNodes;
	currentNodes.push_back(&mMapNodes[mStartPosition.x][mStartPosition.y]);
	currentNodes.at(0)->setF(currentNodes.at(0)->GetH());
	std::vector<Node*> movementOptions;

	while(1)
	{
		// Beam: do the following up to mK times
		for (std::vector<Node*>::size_type k=0; k<currentNodes.size(); k++)
		{
			movementOptions = CalculateMoveOptions(currentNodes.at(k));

			// Beam: Gather movement options of all k branches together into one heap, choose top k for the next iteration
			for (std::vector<Node*>::size_type i=0; i<movementOptions.size();i++)
			{			
				Node::DetailedNodeCompare finder;
				finder.to_find = movementOptions.at(i);

				if ( (find_if(mExpandedNodes.begin(), mExpandedNodes.end(), finder) == mExpandedNodes.end()) &&
						(find_if(mFringeNodes.begin(), mFringeNodes.end(), finder) == mFringeNodes.end()))
				{
					mFringeNodes.push_back(movementOptions.at(i));
					push_heap( mFringeNodes.begin(), mFringeNodes.end(), Node::NodeCompare() ); // Sort back element into Heap
				}
			}
		}

		// No way there, or, there seems to be a problem, exit out before wasting too much time
		if ( (mFringeNodes.size() == 0) || ((int)mExpandedNodes.size() > UPPER_SEARCH_LIMIT))
		{
			mbSuccessfulRun = false;
			break;
		}

		// Clear our current set of nodes for the new wave
		currentNodes.clear();

		// Beam = Pop the mK best nodes (the ones with the lowest f)
		// Add the current nodes to the expanded list.
		for (std::vector<Node*>::size_type k=0; (k< mFringeNodes.size()) && (k<(unsigned)mK); k++)
		{
			currentNodes.push_back(mFringeNodes.at(k));
			push_heap( currentNodes.begin(), currentNodes.end(), Node::NodeCompare() ); // Sort back element into Heap

			// Keep track of expanded nodes for summary data
			mExpandedNodes.push_back(mFringeNodes.at(k));

			pop_heap( mFringeNodes.begin(), mFringeNodes.end(), Node::NodeCompare() );
			mFringeNodes.pop_back();
		}

		// Then clear movementOptions
		movementOptions.clear();

		// Keep only mK values in the fringe
		int size = mFringeNodes.size() - mK;
		for (int k=0; k<(size);k++)
			mFringeNodes.pop_back();

		if (DEBUG && mbVerbose)
		{
			std::cout << "--------------------\n";
			for (std::vector<Node*>::size_type k=0; (k< currentNodes.size()) && (k<(unsigned)mK); k++)
				std::cout << "CNode: (" << currentNodes.at(k)->GetPosition()->x << "," << currentNodes.at(k)->GetPosition()->y << ") g=[" << currentNodes.at(k)->GetG() << 
							"] h=[" << currentNodes.at(k)->GetH() << "] f=" << currentNodes.at(k)->GetF() << "] mov: (" << currentNodes.at(k)->GetMovementVec()->x << "," <<
							currentNodes.at(k)->GetMovementVec()->y << ")\n";
		}

		// End case check
		for (std::vector<Node*>::size_type k=0; (k< currentNodes.size()) && (k<(unsigned)mK); k++)
			if (*currentNodes.at(k)->GetPosition() == mFinishPosition)
				return currentNodes.at(k);
	}

	if (mbVerbose)
		std::cout << "Nodes Expanded= " << mExpandedNodes.size() << std::endl;

	if (currentNodes.size() > 0)
		return currentNodes.at(0);
	else
		return NULL;
}

/// Local Beam Search Implementation
Node* Search::PerformLocalBeamSearch()
{
	// Reset from earlier search runs
	mFringeNodes.clear();
	mExpandedNodes.clear();
	mbSuccessfulRun = true;

	std::vector<Node*> currentNodes;
	currentNodes.push_back(&mMapNodes[mStartPosition.x][mStartPosition.y]);
	currentNodes.at(0)->setF(currentNodes.at(0)->GetH());
	
	std::vector<Node*> movementOptions;

	while(1)
	{
		// Beam: do the following up to mK times
		for (std::vector<Node*>::size_type k=0; k<currentNodes.size(); k++)
		{
			movementOptions = CalculateMoveOptions(currentNodes.at(k));
		
			// Beam: Gather movement options of all k branches together into one heap, choose top k for the next iteration
			for (std::vector<Node*>::size_type i=0; i<movementOptions.size();i++)
			{
				mFringeNodes.push_back(movementOptions.at(i));
				push_heap( mFringeNodes.begin(), mFringeNodes.end(), Node::NodeCompare() ); // Sort back element into Heap
			}
		}

		// No way there, or, there seems to be a problem, exit out before wasting too much time
		if ( (mFringeNodes.size() == 0) || ((int)mExpandedNodes.size() > UPPER_SEARCH_LIMIT/5))
		{
			mbSuccessfulRun = false;
			break;
		}

		// Clear our current set of nodes for the new wave
		currentNodes.clear();

		// Beam = Pop the k best nodes (the ones with the lowest f)
		// Add the current nodes to the expanded list.  Clear the fringe
		for (std::vector<Node*>::size_type k=0; (k< mFringeNodes.size()) && (k<(unsigned)mK); k++)
		{
			currentNodes.push_back(mFringeNodes.at(k));
			push_heap( currentNodes.begin(), currentNodes.end(), Node::NodeCompare() ); // Sort back element into Heap

			// Keep track of expanded nodes for summary data
			mExpandedNodes.push_back(mFringeNodes.at(k));

			pop_heap( mFringeNodes.begin(), mFringeNodes.end(), Node::NodeCompare() );
			mFringeNodes.pop_back();
		}

		// Then clear movementOptions and the fringe
		movementOptions.clear();
		mFringeNodes.clear();

		// End case check
		for (std::vector<Node*>::size_type k=0; (k< currentNodes.size()) && (k<(unsigned)mK); k++)
			if (*currentNodes.at(k)->GetPosition() == mFinishPosition)
				return currentNodes.at(k);
	}

	if (mbVerbose)
		std::cout << "Nodes Expanded= " << mExpandedNodes.size() << std::endl;

	if (currentNodes.size() > 0)
		return currentNodes.at(0);
	else
		return NULL;
}

/// Calculate which nodes are next using current position, movement vector, and map data
std::vector<Node*> Search::CalculateMoveOptions(Node* parent)
{
	std::vector<Node*> currentNodeChoices;
	// Find Our Movement Options given the game rules
	std::vector<Vector2D> movementBreakdown;
	for (int i=0-mMaxDelta; i <= 0+mMaxDelta; i++)
	{
		for (int j=0-mMaxDelta; j <= 0+mMaxDelta; j++)
		{
			Vector2D pos = Vector2D(parent->GetPosition()->x, parent->GetPosition()->y);
			bool bIncludeNode = true;

			if (i == 0 && j == 0)
				bIncludeNode = false;

			if (bIncludeNode)
			{
				pos += Vector2D(i,j);
					
				//Handle the looping map
				if (pos.x < 0)
				{
					pos.x = mWidth-1+pos.x;
				}
				else if (pos.x >= mWidth-1)
				{
					pos.x = pos.x % (mWidth-1);
				}
				if (pos.y < 0)
				{
					pos.y = pos.y * -1;
					pos.x = (mWidth-1) - pos.x;
				}
				else if (pos.y >= mHeight-1)
				{
					pos.y = (mHeight-1) - (pos.y % (mHeight-1));
					pos.x = (mWidth-1) - pos.x;
				}
				
				// Test for water
				if (mMapNodes[pos.x][pos.y].CheckForWater())
				{
					bIncludeNode = false;
					break;
				}
			}

			// If no water was found between start and end position, add as possible Movement Vector
			if (bIncludeNode == true)
			{
				currentNodeChoices.push_back(mMapNodes[pos.x][pos.y].Copy());
				Node* curNode = currentNodeChoices.back();
				Vector2D *mov = new Vector2D(i,j);

				// H(n) = spherical distance to the finish point (x,y) from the current point (i,j)
				float smallestDist;
				if (meSearchType == SEARCH_BEAM || meSearchType == SEARCH_LOCAL)
					smallestDist= (curNode->GetPosition()->CalcSphericalDistanceF(mFinishPosition.x, mFinishPosition.y, mHeight, mWidth)) * 2.0f;
				else
					smallestDist= (curNode->GetPosition()->CalcSphericalDistanceF(mFinishPosition.x, mFinishPosition.y, mHeight, mWidth)) * 2.0f;
				
				curNode->setH(smallestDist);

				// Update f(n), g(n), and movment vector to get here
				curNode->setG(curNode->GetPosition()->CalcSphericalDistanceF(parent->GetPosition()->x, parent->GetPosition()->y, mHeight, mWidth) + parent->GetG());
				curNode->setF(curNode->GetG() + curNode->GetH());
				curNode->SetMovementVec(mov);

				// Add Parent and Children connections
				parent->SetChild(curNode);
				curNode->SetParent(parent);
			}
			movementBreakdown.clear();
		}
	}

	return currentNodeChoices;
}

/// Create the final return path and print to screen if requested
std::vector<osg::Vec2f> Search::ProcessSearchResults(Node* finalNode)
{
	Node* curNode = finalNode;
	std::vector<osg::Vec2f> vPositionList;
	int totalSpeed = 0;
	std::vector<Node*> path;
	
	if (finalNode)
	{
		// Create the path of each point selected
		while (1)
		{
			path.push_back(curNode);
		
			if (curNode->GetParent() == NULL)
				break;
			else
				curNode = curNode->GetParent();
		}
		reverse(path.begin(), path.end());

		// Print status data
		for (std::vector<Node*>::size_type i=0; i<path.size(); i++)
		{
			if (mbVerbose)
			{
				std::cout  << "Step " << i << "\t{P: (" << path.at(i)->GetPosition()->x << "," << path.at(i)->GetPosition()->y << ")\tM:("
						   << path.at(i)->GetMovementVec()->x << "," << path.at(i)->GetMovementVec()->y << ")\tg=" << path.at(i)->GetG()
						   << "\th=" << path.at(i)->GetH() << "\tf=" << path.at(i)->GetF() << "\tSpeed=" << path.at(i)->GetMovementVec()->GetMax()
						   << "}" << std::endl;
			}
		
			vPositionList.push_back(osg::Vec2f((float)path.at(i)->GetPosition()->x/mWidth, (float)path.at(i)->GetPosition()->y/mHeight));
			totalSpeed += path.at(i)->GetMovementVec()->GetMax();
		}

		// Print summary data
		if (mbVerbose)
		{
			std::cout << "Average Speed= " << totalSpeed/path.size() << std::endl;
			std::cout << "Number of Steps= " << path.size() << std::endl;
		}
	}
	
	return vPositionList;
}

/// Calculate linear distance between two points
float Vector2D::CalcLinearDistanceF(int X, int Y)
{
	return sqrtf(((X-x)*(X-x))+((Y-y)*(Y-y)));
}

/// Calculate the spherical distance between two points in U,V space given a map of height and width
float Vector2D::CalcSphericalDistanceF(int X, int Y, int height, int width)
{
	// Spherical Law of Cosines
	float lon1 = osg::DegreesToRadians(((((float)x/width)-0.5f)*360.0f) );	
	float lon2 = osg::DegreesToRadians(((((float)X/width)-0.5f)*360.0f) );	
	float lat1 = osg::DegreesToRadians(((((float)y/height)-0.5f)*180.0f) );
	float lat2 = osg::DegreesToRadians(((((float)Y/height)-0.5f)*180.0f) );
	
	return ( acosf(sinf(lat1)*sinf(lat2) + 
					cosf(lat1)*cosf(lat2) *
					cosf(lon2-lon1)));
	
}

/// Calculate the linear length
float Vector2D::LengthF()
{
	return CalcLinearDistanceF(0,0);
}