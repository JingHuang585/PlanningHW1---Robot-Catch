/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <vector>
#include <iostream>

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
    //Node* temp = *head; 
    (*head) = (*head)->next; 
    //free(temp);
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
    int reach_goal = 0;
    int size = x_size * y_size;
    float w_cost2go = 100;
    float w_cost2come = 1;
    
    std::vector<double> cost2come_array;
    cost2come_array.assign(size, 100000);
    int ori_idx = GETMAPINDEX(robotposeX,robotposeY,x_size,y_size);
    cost2come_array[ori_idx] = 0; //Initialize the start point cost to come to be 0.
    
    std::vector<int>closed_status;
    closed_status.assign(size, 0);
    
    
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    

    int goalposeX = targetposeX;
    int goalposeY = targetposeY;
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);
    
    Node* head = newNode(robotposeX, robotposeY, 0, 0); //Create the open queue nodes.
    
    std::cout << "Thresh is: " << collision_thresh << '\n';
    std::cout << "Current position is: (" << robotposeX << ", " << robotposeY << ");" << '\n';
    std::cout << "Goal position is: (" << goalposeX << ", " << goalposeY << ");" << '\n';
    double disttotarget;
    int i = 0;
    //Plan the whole path based on current location and target location.
    while (reach_goal != 1 && head != nullptr){
        //std::cout << i << '\n';
        if (i > 100000)
            break;
        int* head_pos = peek(&head);
        int cur_X = head_pos[0];
        int cur_Y = head_pos[1];

        int head_idx = GETMAPINDEX(cur_X,cur_Y,x_size,y_size);
        closed_status[head_idx] = 1; //If one node is expanded, set the status to be 1.
        
        head->data[2] = 0;
        head->priority = 0; //Make sure the head is always at the head when adding other nodes.
                            //Initially the head should be removed here. But doing that will cause the head to be a nullptr
                            //for the first iteration. So we cannot remove the head here.
                            //To make sure the head will still be removed after pushing other nodes, the priority of this node 
                            //should be definitly the hightest. Hence set it to be 0.
        
        if (cur_X == goalposeX && cur_Y == goalposeY){
            reach_goal = 1;
            std::cout << "Reaching goal!" << '\n';
        }
        
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = cur_X + dX[dir];
            int newy = cur_Y + dY[dir];

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
                {
                    disttotarget = GETDISTANCE(newx, newy, GOALPOSEX, GOALPOSEY);
                    int idx = GETMAPINDEX(newx,newy,x_size,y_size);
                    double cost2come = w_cost2come * (head_pos[2] + (int)map[idx]);
                    double cost2go = w_cost2go * disttotarget;
                    
                    if (closed_status[idx] == 0){
                        if (cost2come_array[idx] > cost2come_array[head_idx] + cost2come){
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
    else{
        std::cout << "Yes!!!" << '\n';
        std::cout << "Thresh is: " << collision_thresh << '\n';
    }
    
    
    // Trace back.
    while (head->previous->previous != nullptr){
        head = head->previous;
    }
    robotposeX = head->data[0];
    robotposeY = head->data[1];
    
//     robotposeX = robotposeX + bestX;
//     robotposeY = robotposeY + bestY;
    int mapidx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    printf("The map value of next node is %d. \n", (int)map[mapidx]);
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
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