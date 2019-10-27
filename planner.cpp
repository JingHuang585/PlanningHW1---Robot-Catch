/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <vector>
#include <queue>
#include <unordered_map>
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

int GOALX;
int GOALY;
int TIME_DIFF_MARGIN = 100;
typedef pair<int, pair<int,int>> pii;  //First is f value, second is x,y position.

static unordered_map<int, int> parent;                                       //Define the parent map.

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
    int w_cost2go = 1;                                                        //Define the weight of cost to go.
    int ori_idx = GETMAPINDEX(robotposeX,robotposeY,x_size,y_size);           
    parent[ori_idx] = ori_idx;                                                //Initialize the parent of start index to be itself.

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
		vector<int> cost2come_array;                                          //Initialize the time to come array for each grid in map.
		cost2come_array.assign(size, INT_MAX);

		vector<vector<int>> time2come;                                        //Initialize the time to come array for each grid in map.
		time2come.assign(x_size, vector<int>(y_size, INT_MAX));

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
                                parent[idx] = cur_idx;
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

        // Start to do the backtracking.
        int goal_idx = GETMAPINDEX(GOALX, GOALY, x_size, y_size);
        int bt_idx = goal_idx;
        while (parent[parent[bt_idx]] != parent[bt_idx]){
            bt_idx = parent[bt_idx];
        }
        int action_x = bt_idx % x_size + 1;
        int action_y = bt_idx / x_size + 1;
        //cout << "Next position: " << action_x << ", " << action_y << " Index: " << bt_idx << " " << goal_idx << endl;
		action_ptr[0] = action_x;
		action_ptr[1] = action_y;

    }
	else {
		/*This is the A* search to get an optimal path from current position to the goal.
         *To be frankly, we can already get an optimal path from the Dijkstra search in 
         *the preprocess, but I want to also try A* algorithm here. Theoretically, the A* 
         *path should be exactly the same as the Dijkstra path.
         */

        // Start to do the backtracking.
        int goal_idx = GETMAPINDEX(GOALX, GOALY, x_size, y_size);
        int bt_idx = goal_idx;
        while (parent[parent[bt_idx]] != parent[bt_idx]){
            bt_idx = parent[bt_idx];
        }
        int action_x = bt_idx % x_size + 1;
        int action_y = bt_idx / x_size + 1;
        //cout << "Next position: " << action_x << ", " << action_y << " Index: " << bt_idx << " " << goal_idx << endl;
		action_ptr[0] = action_x;
		action_ptr[1] = action_y;
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