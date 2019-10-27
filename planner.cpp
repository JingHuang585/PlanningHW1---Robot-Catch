/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <vector>
#include <queue>
#include <iostream>
#include <chrono> 
#include <algorithm>
#include <numeric>
using namespace std::chrono;
using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#define GETDISTANCE(X, Y, GOALPOSEX, GOALPOSEY) ((double)sqrt(((X-goalposeX)*(X-goalposeX) + (Y-goalposeY)*(Y-goalposeY))))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

typedef struct node { 
    int data[4];
    // Lower values indicate higher priority 
    double priority;
    int t; // Indicates how long this node is reached from start position.
    struct node* next;
    struct node* previous;
  
} Node; 

// Function to Create A New Node 
Node* newNode(int x, int y, double cost2come, double cost2go) 
{ 
    Node* temp = (Node*)malloc(sizeof(Node)); 
    temp->data[0] = x;
    temp->data[1] = y;
    temp->data[2] = cost2come;
    temp->data[3] = cost2go;
    temp->priority = cost2come + cost2go;
    temp->next = NULL;
    temp->previous = NULL;
    
    return temp; 
}

Node* get_head(Node** head){
    //Return the pointer that points to a new node that is the same as the head node.
    Node* temp = new Node;
    temp->data[0] = (*head)->data[0];
    temp->data[1] = (*head)->data[1];
    temp->data[2] = (*head)->data[2];
    temp->data[3] = (*head)->data[3];
    
    temp->priority = (*head)->priority;
    temp->next = (*head)->next;
    temp->previous = (*head)->previous;
    
    return temp;
}
  
// Return the value at head 
int* peek(Node** head) 
{ 
    return (*head)->data; 
}

// Removes the element with the 
// highest priority form the list 
void pop(Node** head) 
{  
    (*head) = (*head)->next; 
}
  
// Function to push according to priority 
void push(Node** head, int x, int y, double cost2come, double cost2go) 
{ 
    Node* start = (*head); 
    double p = cost2come + cost2go;
    // Create new Node 
    Node* temp = newNode(x, y, cost2come, cost2go); 
    temp->previous = *head;
  
    // Special Case: The head of list has lesser 
    // priority than new node. So insert new 
    // node before head node and change head node. 
    if ((*head)->priority > p) {
        // Insert New Node before head
        temp->next = *head; 
        (*head) = temp; 
    }
    else {
        // Traverse the list and find a 
        // position to insert new node 
        while (start->next != NULL && 
               start->next->priority < p) { 
            start = start->next; 
        } 
        // Either at the ends of the list 
        // or at required position 
        temp->next = start->next; 
        start->next = temp; 
    }
}
// Function to check is list is empty 
int isEmpty(Node** head) 
{ 
    return (*head) == NULL; 
} 

int GOALX;
int GOALY;
int TIME_DIFF_MARGIN = 200;
typedef pair<int, pair<int,int>> pii;  //First is cost2come, second is x,y position.
static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    action_ptr[0] = robotposeX;
	action_ptr[1] = robotposeY;
    
    cout << "cur_time is: " << curr_time << endl;
	int size = x_size * y_size;
	
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    if (curr_time == 0){
        /*This is the preprocess to get an optimal goal position.
         *Using forward dijkstra to get the cost from start to every point in the map.
         *Loop through the target_trajectory. Get the time to reach every point on the trajectory,
         *if the time is longer than the time for target to reach that position, it means that point is
         *feasible. 
         */
		auto start = high_resolution_clock::now();
        //Do pre-planning
		vector<int> cost2come_array;                                        //Initialize the time to come array for each grid in map.
		cost2come_array.assign(size, INT_MAX);

		vector<vector<int>> time2come;                                        //Initialize the time to come array for each grid in map.
		time2come.assign(x_size, vector<int>(y_size, INT_MAX));
        int ori_idx = GETMAPINDEX(robotposeX,robotposeY,x_size,y_size);

        cost2come_array[ori_idx] = 0;                                         //Initialize the start point cost to come to be 0.
		time2come[robotposeX][robotposeY] = 0;                                //Initialize the time to come for the starting node.
		vector<int> closed_status;                                            //Initialize closed status array.
		closed_status.assign(size, 0);
        
		priority_queue<pii, vector<pii>, greater<pii>> mq;                                               //Define the priority queue for searching.       
		mq.push({ 0, {robotposeX, robotposeY} });
		
		int count = 0;
        while (!mq.empty()){
			++count;
			//cout << "mq size is: " << mq.size() << endl;

			pii cur = mq.top(); mq.pop();
			int cur_cost2come = cur.first;
			
			int cur_X = cur.second.first; 
			int cur_Y = cur.second.second;
			int cur_time2come = time2come[cur_X][cur_Y];
			int cur_idx = GETMAPINDEX(cur_X, cur_Y, x_size, y_size);
			closed_status[cur_idx] = 1;                                       //Mark this grid to be searched. 
			
			//cout << "cur_X: " << cur_X << " cur_Y: " << cur_Y << endl;
			//cout << "Cur cost2come: " << cur_cost2come << endl;

			for (int dir = 0; dir < NUMOFDIRS; dir++)
			{
				int newx = cur_X + dX[dir];
				int newy = cur_Y + dY[dir];
				//cout << "newx: " << newx << "newy: " << newy << endl;
				//cout << "size: " << size << endl;
				if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
				{
					if ((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh)  //if free
					{
						int idx = GETMAPINDEX(newx, newy, x_size, y_size);
						double cost2come = (int)map[idx];

						if (closed_status[idx] == 0) {
							if (time2come[newx][newy] > cur_time2come + 1) {
								time2come[newx][newy] = cur_time2come + 1;
							}
							if (cost2come_array[idx] > cost2come_array[cur_idx] + cost2come) {
								cost2come_array[idx] = cost2come_array[cur_idx] + cost2come;
								mq.push({ cost2come_array[idx], {newx, newy} });
							}
						}
					}
				}
			}
        }
		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop - start);
		int temp = accumulate(closed_status.begin(), closed_status.end(), 0);
		cout << "total number of searched nodes: " << count << endl;
		cout << "Time for D search: " << duration.count() << endl;
		cout << "Finished D search." << endl;
        for (int i = 0; i < target_steps; ++i){
            int goalposeX = target_traj[i-1];
            int goalposeY = target_traj[i-1+target_steps];
            cout << goalposeX << " " << goalposeY << ": " << time2come[goalposeX][goalposeY] << " " << i << endl;
            if (i - time2come[goalposeX][goalposeY] > TIME_DIFF_MARGIN){
                GOALX = goalposeX;
                GOALY = goalposeY;
                //cout << "GOAL FOUND!: " << GOALX << " " << GOALY << " Index: " << i << endl; 
                break;
            }
        }   
    }
	else {
		int reach_goal = 0;
		//int size = x_size * y_size;
		float w_cost2go = 1;
		float w_cost2come = 1;

		std::vector<double> cost2come_array;
		cost2come_array.assign(size, 100000);
		int ori_idx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
		cost2come_array[ori_idx] = 0; //Initialize the start point cost to come to be 0.

		std::vector<int>closed_status;
		closed_status.assign(size, 0);

		int goalposeX = GOALX;
		int goalposeY = GOALY;
        cout << goalposeX << " " << goalposeY << endl;
        if (robotposeX == goalposeX && robotposeY == goalposeY){
            action_ptr[0] = robotposeX;
		    action_ptr[1] = robotposeY;
            return;
        }

		Node* head = newNode(robotposeX, robotposeY, 0, 0); //Create the open queue nodes.

		//std::cout << "Thresh is: " << collision_thresh << '\n';
		std::cout << "Current position is: (" << robotposeX << ", " << robotposeY << ");" << '\n';
		std::cout << "Goal position is: (" << goalposeX << ", " << goalposeY << ");" << '\n';
		double disttotarget;
		int i = 0;
		//Plan the whole path based on current location and target location.
		while (reach_goal != 1 && head != nullptr) {
			//std::cout << i << '\n';
			if (i > 100000)
				break;
			int* head_pos = peek(&head);
			int cur_X = head_pos[0];
			int cur_Y = head_pos[1];

			int head_idx = GETMAPINDEX(cur_X, cur_Y, x_size, y_size);
			closed_status[head_idx] = 1; //If one node is expanded, set the status to be 1.

			head->data[2] = 0;
			head->priority = 0; //Make sure the head is always at the head when adding other nodes.
								//Initially the head should be removed here. But doing that will cause the head to be a nullptr
								//for the first iteration. So we cannot remove the head here.
								//To make sure the head will still be removed after pushing other nodes, the priority of this node 
								//should be definitly the highest. Hence set it to be 0.

			if (cur_X == goalposeX && cur_Y == goalposeY) {
				reach_goal = 1;
                break;
				//std::cout << "Reaching goal!" << '\n';
			}

			for (int dir = 0; dir < NUMOFDIRS; dir++)
			{
				int newx = cur_X + dX[dir];
				int newy = cur_Y + dY[dir];

				if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
				{
					if (((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] >= 0) && ((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh))  //if free
					{
						disttotarget = GETDISTANCE(newx, newy, GOALPOSEX, GOALPOSEY);
						int idx = GETMAPINDEX(newx, newy, x_size, y_size);
						double cost2come = w_cost2come * (head_pos[2] + (int)map[idx]);
						double cost2go = w_cost2go * disttotarget;

						if (closed_status[idx] == 0) {
							if (cost2come_array[idx] > cost2come_array[head_idx] + cost2come) {
								cost2come_array[idx] = cost2come_array[head_idx] + cost2come;
								push(&head, newx, newy, cost2come, cost2go);
							}
						}
					}
				}
			}
			pop(&head); //Remove the head of the open queue.
			++i;
		}
		//printf("The reach_goal is: %d ", reach_goal);
		if (head == nullptr)
			std::cout << "The open queue becomes nullptr." << '\n';
		else {
			//std::cout << "Yes!!!" << '\n';
			//std::cout << "Thresh is: " << collision_thresh << '\n';
		}


		// Trace back.
		while (head->previous->previous != nullptr) {
			head = head->previous;
		}
		robotposeX = head->data[0];
		robotposeY = head->data[1];

		int mapidx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
		//printf("The map value of next node is %d. \n", (int)map[mapidx]);
		action_ptr[0] = robotposeX;
		action_ptr[1] = robotposeY;
		
	}
    
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}