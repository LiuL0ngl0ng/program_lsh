
#include <iostream>
#include <stdio.h>
#include <assert.h>
// stl includes
#include <algorithm>
#include <set>
#include <vector>

#include <QList>


using namespace std;
// fast fixed size memory allocator, used for fast node memory management
#include "fsa.h"
// Fixed size memory allocator can be disabled to compare performance
// Uses std new and delete instead if you turn it off
#define USE_FSA_MEMORY 1
// disable warning that debugging information has lines that are truncated
// occurs in stl headers
//#pragma warning( disable : 4786 )         //notice

// The AStar search class. UserState is the users state space type
template <class UserState> class AStarSearch//lsh//创建模板类，UserState为待定数据类型
{
public: // data//lsh//枚举模式
    enum
    {
        SEARCH_STATE_NOT_INITIALISED,
        SEARCH_STATE_SEARCHING,
        SEARCH_STATE_SUCCEEDED,
        SEARCH_STATE_FAILED,
        SEARCH_STATE_OUT_OF_MEMORY,
        SEARCH_STATE_INVALID
    };
    // A node represents a possible state in the search
    // The user provided state type is included inside this type
public://lsh//a*中每个节点的信息
    class Node
    {
        public:
            Node *parent; // used during the search to record the parent of successor nodes
            Node *child; // used after the search for the application to view the search in reverse
            float g; // cost of this node + it's predecessors
            float h; // heuristic estimate of distance to goal
            float f; // sum of cumulative cost of predecessors and self and heuristic
            UserState *m_UserState;
        //lsh//实际定义的用户类型为MapSearchNode，存储的是每个路点的位置、属性等
            Node() :
                parent( 0 ),
                child( 0 ),
                g( 0.0f ),//lsh//0.0f表示单精度浮点数
                h( 0.0f ),
                f( 0.0f )
            {
            }
    };

    // For sorting the heap the STL needs compare function that lets us compare
    // the f value of two nodes
    class HeapCompare_f//lsh//比较两个节点的代价
    {
        public:
            bool operator() ( const Node *x, const Node *y ) const
            {
                return x->f > y->f;
            }
    };
public: // methods
    // constructor just initialises private data
    AStarSearch( int MaxNodes = 10000 ) ://lsh//分配足够的存储空间，初始化
        m_AllocateNodeCount(0),
#if USE_FSA_MEMORY//lsh//如果为真则编译#if USE_FSA_MEMORY与#endifg之间的内容
        m_FixedSizeAllocator( MaxNodes ),//lsh//分配了MaxNodes个Node空间
#endif
        m_State( SEARCH_STATE_NOT_INITIALISED ),//lsh//搜索状态未初始化模式
        m_CurrentSolutionNode( NULL ),
        m_CancelRequest( false )
        {
        }
    // call at any time to cancel the search and free up all the memory
    void CancelSearch()
    {
        m_CancelRequest = true;
    }
    // Set Start and goal states
    void SetStartAndGoalStates( UserState *Start, UserState *Goal )
    {//lsh//设置起止点，将m_Start放入m_OpenList进行堆排序
        m_CancelRequest = false;
        m_Start = AllocateNode();//给其分配了一块空间
        m_Goal = AllocateNode();
        assert((m_Start != NULL && m_Goal != NULL));
        //lsh//assert的作用是先判断，如果其值为假（即为0），那么它先向stderr打印一条出错信息，然后通过调用abort来终止程序运行。
        m_Start->m_UserState = Start;
        m_Goal->m_UserState = Goal;
        m_State = SEARCH_STATE_SEARCHING;
        // Initialise the AStar specific parts of the Start Node
        // The user only needs fill out the state information
        m_Start->g = 0;
        m_Start->h = m_Start->m_UserState->GoalDistanceEstimate( m_Goal->m_UserState );
        m_Start->f = m_Start->g + m_Start->h;
        m_Start->parent = 0;
        // Push the start node on the Open list
        m_OpenList.push_back( m_Start ); // heap now unsorted//lsh//将一个新的元素加到vector的最后面
        // Sort back element into heap
        push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
        //lsh//堆是一种数据结构，就是每个节点根据某种规则排序， 从根节点往下都符合某种规律，
        //lsh//根节点的值比所有节点的值都大， 称为最大堆；
        //lsh//根节点的值比所有节点的值都小， 称为最小堆；（此处貌似为最小堆）
        //lsh//push_heap()是向堆中插入一个元素，并且使堆的规则依然成立
        // Initialise counter for search steps
        m_Steps = 0;
    }
    // Advances search one step
    unsigned int SearchStep()
    {
        // Firstly break if the user has not initialised the search
        assert( (m_State > SEARCH_STATE_NOT_INITIALISED) &&
                (m_State < SEARCH_STATE_INVALID) );
        //lsh//assert() 的用法像是一种"契约式编程"，程序在假设条件下，能够正常良好的运作，否则报错并终止程序
        // Next I want it to be safe to do a searchstep once the search has succeeded...
        if( (m_State == SEARCH_STATE_SUCCEEDED) ||(m_State == SEARCH_STATE_FAILED) )
        {
            return m_State;
        }
        // Failure is defined as emptying the open list as there is nothing left to
        // search...
        // New: Allow user abort
        if( m_OpenList.empty() || m_CancelRequest )
        {//lsh//若m_OpenList为空或m_CancelRequest为真，则清空内存返回搜做失败
            FreeAllNodes();
            m_State = SEARCH_STATE_FAILED;
            return m_State;
        }
        // Incremement step count
        m_Steps ++;
        // Pop the best node (the one with the lowest f)
        Node *n = m_OpenList.front(); // get pointer to the node
        pop_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
        //lsh//pop_heap()是在堆的基础上，弹出堆顶元素。
        m_OpenList.pop_back();//lsh//删除这个元素


        // Check for the goal, once we pop that we're done
        if( n->m_UserState->IsGoal( m_Goal->m_UserState ) )
        {
            // The user is going to use the Goal Node he passed in
            // so copy the parent pointer of n
            m_Goal->parent = n->parent;
            // A special case is that the goal was passed in as the start state
            // so handle that here
            if( false == n->m_UserState->IsSameState( m_Start->m_UserState ) )
            {
                FreeNode( n );
                // set the child pointers in each node (except Goal which has no child)
                Node *nodeChild = m_Goal;
                Node *nodeParent = m_Goal->parent;
                do
                {
                    nodeParent->child = nodeChild;
                    nodeChild = nodeParent;
                    nodeParent = nodeParent->parent;
                }
                while( nodeChild != m_Start ); // Start is always the first node by definition
            }//lsh//根据搜索过程中设置的父节点关系设置子节点关系
            // delete nodes that aren't needed for the solution
            FreeUnusedNodes();
            m_State = SEARCH_STATE_SUCCEEDED;
            return m_State;
        }
        else // not goal
        {
            // We now need to generate the successors of this node
            // The user helps us to do this, and we keep the new nodes in
            // m_Successors ...
            m_Successors.clear(); // empty vector of successor nodes to n
            // User provides this functions and uses AddSuccessor to add each successor of
            // node 'n' to m_Successors
            {
                //POSITION pos=n->m_UserState->successorList.GetHeadPosition();
                typename QList<UserState*>::iterator pos=n->m_UserState->successorList.begin();      //tangbo
                for(;pos!=n->m_UserState->successorList.end();pos++)                    //靠距离最小找出所选点
                {
                //while(pos!=NULL)
                //{
                //	AddSuccessor((UserState*)n->m_UserState->successorList.GetNext(pos));
                    AddSuccessor((UserState*)(*pos));      //tangbo
                }
            }
            //lsh//找到所取出点的子节点，加入到m_Successors
            //if( 0 )
            //{
            //    typename vector< Node * >::iterator successor;
            //	// free the nodes that may previously have been added
            //	for( successor = m_Successors.begin(); successor != m_Successors.end(); successor ++ )
            //	{
            //		FreeNode( (*successor) );
            //	}
            //	m_Successors.clear(); // empty vector of successor nodes to n
            //	// free up everything else we allocated
            //	FreeAllNodes();
            //	m_State = SEARCH_STATE_OUT_OF_MEMORY;
            //	return m_State;
            //}
            // Now handle each successor to the current node ...
            for( typename vector< Node * >::iterator successor = m_Successors.begin(); successor != m_Successors.end(); successor ++ )
            {
                // 	The g value for this successor ...
                float newg = n->g + n->m_UserState->GetCost( (*successor)->m_UserState );//GetCost返回n到这个子节点的欧式距离
                //lsh//得到每个子节点的g值
                //lsh//此时计算的是距离取出点的dx+dy
                // Now we need to find whether the node is on the open or closed lists
                // If it is but the node that is already on them is better (lower g)
                // then we can forget about this successor
                // First linear search of open list to find node
                typename vector< Node * >::iterator openlist_result;
                for( openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result ++ )
                {
                    if( (*openlist_result)->m_UserState->IsSameState( (*successor)->m_UserState ) )     //notice
                    {
                        break;
                    }
                }//lsh//判断子节点是不是已经在m_OpenList中
                if( openlist_result != m_OpenList.end() )
                {
                    //说明存在在openlist中
                    if( (*openlist_result)->g <= newg )
                    {
                        FreeNode( (*successor) );
                        // the one on Open is cheaper than this one
                        continue;//lsh//判断下一个子节点
                    }
                }

                typename vector< Node * >::iterator closedlist_result;
                for( closedlist_result = m_ClosedList.begin(); closedlist_result != m_ClosedList.end(); closedlist_result ++ )
                {
                    if( (*closedlist_result)->m_UserState->IsSameState( (*successor)->m_UserState ) )
                    {
                        break;
                    }
                }//lsh//判断子节点是不是已经在m_ClosedList中
                if( closedlist_result != m_ClosedList.end() )
                {
                    // we found this state on closed
                    if( (*closedlist_result)->g <= newg )
                    {
                        // the one on Closed is cheaper than this one
                        FreeNode( (*successor) );
                        continue;
                    }
                }
                // This node is the best node so far with this particular state
                // so lets keep it and set up its AStar specific data ...
                //lsh//当前保留的子节点为未重复，或重复了但最优的节点，为其设置父节点并设置g、h、f值
                (*successor)->parent = n;
                (*successor)->g = newg;
                (*successor)->h = (*successor)->m_UserState->GoalDistanceEstimate( m_Goal->m_UserState );
                (*successor)->f = (*successor)->g + (*successor)->h;
                // Remove successor from closed if it was on it
                if( closedlist_result != m_ClosedList.end() )
                {//lsh//结合前面的if( closedlist_result != m_ClosedList.end() )说明此时拓展的子节点比之前访问过的节点要好，删除之前访问记录
                    // remove it from Closed
                    FreeNode(  (*closedlist_result) );
                    m_ClosedList.erase( closedlist_result );
                    //lsh//从m_ClosedList中删除该访问过的节点
                }
                // Update old version of this node
                if( openlist_result != m_OpenList.end() )
                {//lsh//结合之前的if( openlist_result != m_OpenList.end() )说明此时拓展的子节点比之前m_OpenList中的节点更优
                    FreeNode( (*openlist_result) );
                    m_OpenList.erase( openlist_result );
                    make_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
                }
                // heap now unsorted
                m_OpenList.push_back( (*successor) );
                // sort back element into heap
                push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
                //lsh//删除之前的节点，加入此拓展节点并进行排序
            }
            // push n onto Closed, as we have expanded it now
            m_ClosedList.push_back( n );
        } // end else (not goal so expand)
        return m_State; // Succeeded bool is false at this point.
    }
    // User calls this to add a successor to a list of successors
    // when expanding the search frontier
    bool AddSuccessor( UserState *State )
    {//lsh//把State移动到m_Successors中
        Node *node = AllocateNode();
        if( node )
        {
            node->m_UserState = State;
            m_Successors.push_back( node );
            return true;
        }
        return false;
    }
    // Free the solution nodes
    // This is done to clean up all used Node memory when you are done with the
    // search
    void FreeSolutionNodes()
    {
        Node *n = m_Start;
        if( m_Start->child )
        {
            do
            {
                Node *del = n;
                n = n->child;
                FreeNode( del );
                del = NULL;
            } while( n != m_Goal );
            FreeNode( n ); // Delete the goal
        }
        else
        {
            // if the start node is the solution we need to just delete the start and goal
            // nodes
            FreeNode( m_Start );
            FreeNode( m_Goal );
        }
    }
    // Functions for traversing the solution

    // Get start node
    UserState *GetSolutionStart()
    {//lsh//将m_CurrentSolutionNode设为起点，并返回起点
        m_CurrentSolutionNode = m_Start;
        if( m_Start )
        {
            return m_Start->m_UserState;
        }
        else
        {
            return NULL;
        }
    }
    // Get next node
    UserState *GetSolutionNext()
    {//lsh//将m_CurrentSolutionNode设为下一个点，并返回下一个点
        if( m_CurrentSolutionNode )
        {
            if( m_CurrentSolutionNode->child )
            {
                Node *child = m_CurrentSolutionNode->child;
                m_CurrentSolutionNode = m_CurrentSolutionNode->child;
                return child->m_UserState;
            }
        }
        return NULL;
    }
    // Get end node
    UserState *GetSolutionEnd()
    {
        m_CurrentSolutionNode = m_Goal;
        if( m_Goal )
        {
            return m_Goal->m_UserState;
        }
        else
        {
            return NULL;
        }
    }
    //
    //void GetSolutionNodeList(CPtrList *SolutionNodeList)
    void GetSolutionNodeList(QList<UserState*> *SolutionNodeList)       //tangbo
    {//lsh//生成最优路点表单
        UserState *first_node = GetSolutionStart();
        while(first_node)
        {
            SolutionNodeList->append(first_node);
            first_node = GetSolutionNext();
        }
    }

    // Step solution iterator backwards
    UserState *GetSolutionPrev()
    {
        if( m_CurrentSolutionNode )
        {
            if( m_CurrentSolutionNode->parent )
            {
                Node *parent = m_CurrentSolutionNode->parent;
                m_CurrentSolutionNode = m_CurrentSolutionNode->parent;
                return parent->m_UserState;
            }
        }
        return NULL;
    }
    // For educational use and debugging it is useful to be able to view
    // the open and closed list at each step, here are two functions to allow that.
    UserState *GetOpenListStart()
    {//lsh//得到m_OpenList最前面的节点的f、g、h值，返回其节点
        float f,g,h;
        return GetOpenListStart( f,g,h );
    }
    UserState *GetOpenListStart( float &f, float &g, float &h )
    {
        iterDbgOpen = m_OpenList.begin();
        if( iterDbgOpen != m_OpenList.end() )
        {
            f = (*iterDbgOpen)->f;
            g = (*iterDbgOpen)->g;
            h = (*iterDbgOpen)->h;
            return (*iterDbgOpen)->m_UserState;
        }
        return NULL;
    }
    UserState *GetOpenListNext()
    {
        float f,g,h;
        return GetOpenListNext( f,g,h );
    }
    UserState *GetOpenListNext( float &f, float &g, float &h )
    {//lsh//得到m_OpenList下一个的节点的f、g、h值，返回其节点
        iterDbgOpen++;
        if( iterDbgOpen != m_OpenList.end() )
        {
            f = (*iterDbgOpen)->f;
            g = (*iterDbgOpen)->g;
            h = (*iterDbgOpen)->h;
            return (*iterDbgOpen)->m_UserState;
        }
        return NULL;
    }
    UserState *GetClosedListStart()
    {
        float f,g,h;
        return GetClosedListStart( f,g,h );
    }
    UserState *GetClosedListStart( float &f, float &g, float &h )
    {//lsh//得到m_OpenList最前面的节点的f、g、h值，返回其节点
        iterDbgClosed = m_ClosedList.begin();
        if( iterDbgClosed != m_ClosedList.end() )
        {
            f = (*iterDbgClosed)->f;
            g = (*iterDbgClosed)->g;
            h = (*iterDbgClosed)->h;
            return (*iterDbgClosed)->m_UserState;
        }
        return NULL;
    }
    UserState *GetClosedListNext()
    {
        float f,g,h;
        return GetClosedListNext( f,g,h );
    }
    UserState *GetClosedListNext( float &f, float &g, float &h )
    {//lsh//得到m_OpenList下一个的节点的f、g、h值，返回其节点
        iterDbgClosed++;
        if( iterDbgClosed != m_ClosedList.end() )
        {
            f = (*iterDbgClosed)->f;
            g = (*iterDbgClosed)->g;
            h = (*iterDbgClosed)->h;

            return (*iterDbgClosed)->m_UserState;
        }
        return NULL;
    }
    // Get the number of steps
    int GetStepCount() { return m_Steps; }
    void EnsureMemoryFreed()
    {
        #if USE_FSA_MEMORY
                assert(m_AllocateNodeCount == 0);
        #endif
    }
private: // methods
    // This is called when a search fails or is cancelled to free all used
    // memory
    void FreeAllNodes()
    {//lsh//清空m_OpenList与m_ClosedList，打断m_Goal的连接关系
        // iterate open list and delete all nodes
        typename vector< Node * >::iterator iterOpen = m_OpenList.begin();
        //lsh//定义了一个具有Node *元素的迭代器类型（类似于m_OpenList内部元素的指针）
        while( iterOpen != m_OpenList.end() )
        {
            Node *n = (*iterOpen);
            FreeNode( n );
            iterOpen ++;
        }
        m_OpenList.clear();
        // iterate closed list and delete unused nodes
        typename vector< Node * >::iterator iterClosed;
        for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed ++ )
        {
            Node *n = (*iterClosed);
            FreeNode( n );
        }
        m_ClosedList.clear();
        // delete the goal
        FreeNode(m_Goal);
    }
    // This call is made by the search class when the search ends. A lot of nodes may be
    // created that are still present when the search ends. They will be deleted by this
    // routine once the search ends
    void FreeUnusedNodes()
    {//删除m_OpenList和m_ClosedList中不需要的点
        // iterate open list and delete unused nodes
        typename vector< Node * >::iterator iterOpen = m_OpenList.begin();
        while( iterOpen != m_OpenList.end() )
        {
            Node *n = (*iterOpen);
            if( !n->child )
            {
                FreeNode( n );
                n = NULL;
            }
            iterOpen ++;
        }
        m_OpenList.clear();
        // iterate closed list and delete unused nodes
        typename vector< Node * >::iterator iterClosed;
        for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed ++ )
        {
            Node *n = (*iterClosed);
            if( !n->child )
            {
                FreeNode( n );
                n = NULL;
            }
        }
        m_ClosedList.clear();
    }
    // Node memory management
    Node *AllocateNode()
    {//lsh//分配一个存储空间
        #if !USE_FSA_MEMORY
                Node *p = new Node;
                return p;
        #else
            Node *address = m_FixedSizeAllocator.alloc();
            //lsh//从分配的m_pFirstFree中拿出一个存储空间给address，并将该存储空间设为使用过的
            if( !address )
            {
                return NULL;
            }
            m_AllocateNodeCount ++;
            Node *p = new (address) Node;//lsh//把返回的节点地址作为当前的地址
            return p;
        #endif
    }
    void FreeNode( Node *node )
    {//lsh//把节点node从连接关系中删除
     //lsh//m_pFirstFree存储被删除的节点，加到其最前面
        m_AllocateNodeCount --;
        #if !USE_FSA_MEMORY
                delete node;
        #else
                m_FixedSizeAllocator.free( node );
        #endif
    }
private: // data
    // Heap (simple vector but used as a heap, cf. Steve Rabin's game gems article)
    vector< Node *> m_OpenList;
    // Closed list is a vector.
    vector< Node * > m_ClosedList;
    // Successors is a vector filled out by the user each type successors to a node
    // are generated
    vector< Node * > m_Successors;
    // State
    unsigned int m_State;
    // Counts steps
    int m_Steps;
    // Start and goal state pointers
    Node *m_Start;
    Node *m_Goal;
    Node *m_CurrentSolutionNode;

    Node *Cur_Node;
    Node *Next_Node;
    //CPtrList SuccessorList;
    QList<UserState*> SuccessorList;        //tangbo
public:
    //CPtrList NodeList;
    QList<UserState*> NodeList;     //tangbo
#if USE_FSA_MEMORY
    // Memory
    FixedSizeAllocator<Node> m_FixedSizeAllocator;
#endif

    //Debug : need to keep these two iterators around
    // for the user Dbg functions
    typename vector< Node * >::iterator iterDbgOpen;
    typename vector< Node * >::iterator iterDbgClosed;

    // debugging : count memory allocation and free's
    int m_AllocateNodeCount;

    bool m_CancelRequest;

};




