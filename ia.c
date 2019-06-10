// ia.c
// Helper functions to handle robot navigation
// & objective selection
// ---------------------------------------------------
// (C) Martin Raynal, 2014
// NeanderBot 2014

#include <stdio.h>
#include <stdlib.h>

#define INF 10000 //infinity
#define NB_SOMMET 7

typedef enum WayPoints
{
    STARTPOINT = 0,
    FRESCO_TURN = 1,
    FRESCO = 2,
    FRESCO_BACK = 3,
    FIRE_TURN = 4,
    FIRE = 5,
    LAUNCHPAD = 6
} Waypoints;

typedef enum Heading
{
    D_180 = 180,
    D_MINUS_90 = -90,
    D_0 = 0,
    D_PLUS_90 = 90

} Heading;

// Robot status globals
static int currentPoint;
static Heading theta;

typedef struct Link {
    int start;
    int stop;    //the following node index
    int dist;           //distance between nodes
    Heading orientation;

    //void* action ///TODO action to perform before move (ie, disable sonar)

} Link;

int executeMove(Link l)
{
    ///TODO
    //execute pre-move action if it exists
    //if stg prevents further execution, cancels

    // if(blocked)
    //     return -1;

    printf("> Evaluating Turn\n  Current: %d; Desired: %d; Sum: %d\n",theta, l.orientation, l.orientation-theta);

    switch (l.orientation-theta)
    {
    case 270:
    case -90:
        printf("TURN -90\n");
        break;
    case 180:
    case -180:
        printf("TURN 180\n");
        break;
    case -270:
    case 90:
        printf("TURN 90\n");
        break;
    case 0: //aligned
        //ready to move !;
        break;
    }
    theta = l.orientation;

    //Move now that robot is facing good direction
    printf("MOVE %d\n", l.dist);
}

//=========================================================
//  DIJKSTRA
//=========================================================
// Pompé de http://blog.developpez.com/nico-pyright/p9300/cc-win32/c_implementation_de_l_algo_de_dijkstra_e

// Playfield modelisation
//   Adjacency matrix
int map[NB_SOMMET][NB_SOMMET] =
        //FROM
 //0 | 1 | 2 | 3 | 4 | 5 | 6
{{ 0 , 1 ,INF,INF, 1 ,INF, 1 },
 { 1 , 0 ,INF, 1 , 1 ,INF, 1 },
 {INF, 1 , 0 ,INF,INF,INF,INF},  //T
 {INF,INF, 1 , 0 ,INF,INF,INF},  //O
 { 1 , 1 ,INF,INF, 0 , 1 , 1 },
 {INF,INF,INF,INF, 1 , 0 ,INF},
 { 1 , 1 ,INF,INF, 1 ,INF, 0 }};

//   Specifics of all links
Link links[NB_SOMMET][NB_SOMMET];


void dijkstra(int sr,int ds, int path[])
{
    if (sr == ds)
        return;

    printf(" > Applying DIJKSTRA from %d to %d.\n",sr, ds);

    typedef enum Label{
        perm,tent
    }Label;

    struct node
    {
        int pre;   /* Predecessor */
        int length; /* Length between the nodes */
        Label label; /* Enumeration for permanent and tentative labels */
    }state[NB_SOMMET];

    int i,k,min;
    struct node *p;
    /* Initialisation of the nodes aka First step of Dijkstra Algo */
    for(p=&state[0];p<&state[NB_SOMMET];p++)
    {
        p->pre= -1;
        p->length=INF;
        p->label=tent;
    }
    state[ds].length=0; /* Destination length set to zero */
    state[ds].label=perm; /* Destination set to be the permanent node */
    k=ds; /* initial working node */
    /* Checking for a better path from the node k ? */
    do
    {
        for(i=0;i<NB_SOMMET;i++)
        {
            if(map[k][i]!=0 && state[i].label==tent)
            {
                if((state[k].length+map[k][i])<state[i].length)
                {
                    state[i].pre=k;
                    state[i].length=state[k].length+map[k][i];
                }
            }
        }
        k=0;
        min=INF;
        /* Find a node which is tentatively labeled and with minimum label */
        for(i=0;i<NB_SOMMET;i++)
        {
            if(state[i].label==tent && state[i].length<min)
            {
                min=state[i].length;
                k=i;
            }
        }
        state[k].label=perm;

        //printf(" >>> Am I in an infinite loop ? %d != %d.\n",sr, k);
    } while(k!=sr);

    i=0;
    k=sr;
    /* Print the path to the output array */
    do
    {
        path[i++]=k;
        k=state[k].pre;
    } while(k>=0);
    path[i]=k;
}

//=========================================================
//  OBJECTIVES
//=========================================================
#define COEF_BASE 2
//#define COEF_DIST ? //to be tuned...
#define COEF_PRIO 10 //For now, with so little objectives,
// manual assignment shall have precedence.

typedef struct Objective
{
    int baseScore;
    double successProbability;
    int wayPoint;
    int priority;

    char done;
    char blocked; //this field should be set to true when failing to reach current,
    //and unset for every obj when current is completed
}Objective;

/// If all goals are done, blocked or unattainable in time, returns -1
/// When returning -1 main program should unset all blocked flags & retry
/// if really all are done, robot should return in start area
int selectNextObjective(Objective goals[], int goalCount)
{
    int retVal = -1;
    double maxScore = 0;
    printf("Evaluating objectives... (%d,%f)\n",retVal,maxScore);

    int i;
    for (i=0; i<goalCount; i++)
    {
        if(goals[i].done || goals[i].blocked)
            continue;

        double score = COEF_BASE * goals[i].baseScore * goals[i].successProbability;
        score += COEF_PRIO * goals[i].priority;
        /// TODO: add parameters to calculation to
        /// take in account distance & remaining time
        /// (i.e, do I have time to perform high value action ? if not, can I still score low value pts?)

        if(score > maxScore)
        {
            maxScore=score;
            retVal = i;
        }
        printf("Goal %d: score %f (current goal: %d)\n", i,score,retVal);
    }

    printf("Selected goal %d.\n",retVal);
    return retVal; //index of next goal

}


void markDone(int goal,Objective goals[], int goalCount) //NOTE: could be simplified using a global[] & define count
{
    goals[goal].done = 1;
    int i;
    // unblock all
    for(i=0; i<goalCount;i++)
        goals[i].blocked = 0;
}

//=========================================================
//  TESTS
//=========================================================
void printPath(int path[])
{
    int i;
    printf("%d", path[0]);
    for(i = 1; i<50 && path[i] != -1; i++)
    {
        printf(" > %d", path[i]);
    }
    printf("#\n");
}

void execPath(int path[])
{
    int i;
    for(i = 1; i<50 && path[i] != -1; i++)
    {
        printf(":%d(%d) > %d(%d)\n", i-1,links[i-1][i].start, i,links[i-1][i].stop);
        executeMove(links[i-1][i]);
    }
    printf("#DONE#\n");
}


//    5
//    ^
//    v
//0<=>4<=>6<=>1<-+
//            v  |
//            2->3
//
// (0,4,6 & 1 are all interlinked)
// (1>2>3>1 is a unidirectionnal loop)
void hookUpLinks()
{
    Link l1;
    // 0 > 1, 4, 6
    l1.start = 0;
    l1.stop = 1;
    l1.dist = 1210;
    l1.orientation = D_0;
    links[0][1] = l1;

    Link l2;
    l2.start = 0;
    l2.stop = 4;
    l2.dist = 330;//?
    l2.orientation = D_0;
    links[0][4] = l2;

    Link l3;
    l3.start = 0;
    l3.stop = 6;
    l3.dist = 600; // ?
    l3.orientation = D_0;
    links[0][6] = l3;
    //-------------------------------
    // 1 > 0,2,4,6
    Link l4;
    l4.start = 1;
    l4.stop = 0;
    l4.dist = 1210;
    l4.orientation = D_180;
    links[1][0] = l4;

    Link l5;
    l5.start = 1;
    l5.stop = 2;
    l5.dist = 600;
    l5.orientation = D_PLUS_90;
    links[1][2] = l5;

    Link l6;
    l6.start = 1;
    l6.stop = 4;
    l6.dist = 880;
    l6.orientation = D_180;
    links[1][4] = l6;

    Link l7;
    l7.start = 1;
    l7.stop = 6;
    l7.dist = 610;//?
    l7.orientation = D_180;
    links[1][6] = l7;
    //-------------------------------
    //2 > 3 // /!\ must be done reverse !!!
    Link l8;
    l8.start = 2;
    l8.stop = 3;
    l8.dist = -150;
    l8.orientation = D_PLUS_90; //+90 & negative dist
    links[2][3] = l8;
    //-------------------------------
    //3 > 1
    Link l9;
    l9.start = 3;
    l9.stop = 1;
    l9.dist = 400;
    l9.orientation = D_MINUS_90;
    links[3][1] = l9;
    //-------------------------------
    // 4 > 0,1,5,6
    Link l10;
    l10.start = 4;
    l10.stop = 0;
    l10.dist = 330; //?
    l10.orientation = D_180;
    links[4][0] = l10;

    Link l11;
    l11.start = 4;
    l11.stop = 1;
    l11.dist = 880;
    l11.orientation = D_0;
    links[4][1] = l11;

    Link l12;
    l12.start = 4;
    l12.stop = 5;
    l12.dist = 700;
    l12.orientation = D_MINUS_90;
    links[4][5] = l12;

    Link l13;
    l13.start = 4;
    l13.stop = 6;
    l13.dist = 200; //?
    l13.orientation = D_0;
    links[4][6] = l13;
    //-------------------------------
    // 5 > 4
    Link l14;
    l14.start = 5;
    l14.stop = 4;
    l14.dist = 700;
    l14.orientation = D_PLUS_90;
    links[5][4] = l14;
    //-------------------------------
    // 6 > 0,1,4
    Link l15;
    l15.start = 6;
    l15.stop = 0;
    l15.dist = 600;
    l15.orientation = D_180;
    links[6][0] = l15;

    Link l16;
    l16.start = 6;
    l16.stop = 1;
    l16.dist = 610;//?
    l16.orientation = D_0;
    links[6][1] = l16;

    Link l17;
    l17.start = 6;
    l17.stop = 4;
    l17.dist = 200;//?
    l17.orientation = D_180;
    links[6][4] = l17;

}

int main(void)
{
    printf("Start\n");
    //robot position parameters
    currentPoint = STARTPOINT;
    theta = D_0;

    //defineGoals();
    Objective Goals[3];
    Goals[0].baseScore = 6+1; //fresque + fire
    Goals[0].successProbability = 1.0;
    Goals[0].wayPoint = FRESCO_BACK;
    Goals[0].priority = 10;
    Goals[0].done = 0;
    Goals[0].blocked = 0;
    Goals[1].baseScore = 1; //fire
    Goals[1].successProbability = 1.0;
    Goals[1].wayPoint = FIRE;
    Goals[1].priority = 5;
    Goals[1].done = 0;
    Goals[1].blocked = 0;
    Goals[2].baseScore = 3*6; //balls
    Goals[2].successProbability = 0.5; //to be tuned; selecting one ball over two for now
    Goals[2].wayPoint = LAUNCHPAD;
    Goals[2].priority = 1;
    Goals[2].done = 0;
    Goals[2].blocked = 0;

    hookUpLinks();

    //init path array
    int path[50];
    int z;
    for (z = 0 ; z < 50 ; z++)
        path[z] = -1;

    printf("Start to fresco\n");
    dijkstra(STARTPOINT, FRESCO_BACK, path);
    printPath(path);
    printf("Fire to fresco\n");
    dijkstra(FIRE, FRESCO_BACK, path);
    printPath(path);
    printf("Lauch point to fresco\n");
    dijkstra(LAUNCHPAD, FRESCO_BACK, path);
    printPath(path);
    //    printf("fresco to fire\n");
    //    dijkstra(FRESCO_BACK, FIRE, path);
    //    printPath(path);

    printf("\n\nSelector test\n-----------------------------\n");
    printf("Pos: %d; Th: %d\n", currentPoint, theta);
    int target = selectNextObjective(Goals,3);
    dijkstra(currentPoint, Goals[target].wayPoint, path);
    currentPoint = Goals[target].wayPoint;
    printPath(path);
    execPath(path);
    printf("Reached target: %d\n", Goals[target].wayPoint);
    printf("Pos: %d; Th: %d\n", currentPoint, theta);
    markDone(target,Goals,3);

    printf("-----------------------------\n");
    target = selectNextObjective(Goals,3);
    dijkstra(currentPoint, Goals[target].wayPoint, path);
    currentPoint = Goals[target].wayPoint;
    printPath(path);
    execPath(path);
    printf("Reached target: %d\n", target);
    printf("Pos: %d; Th: %d\n", currentPoint, theta);
    markDone(target,Goals,3);

    target = selectNextObjective(Goals,3);
    dijkstra(currentPoint, Goals[target].wayPoint, path);
    //TODO: execution of path
    //> update heading by TURN then MOVE
    //  using waypoints data
    currentPoint = Goals[target].wayPoint;
    printPath(path);
    execPath(path);
    printf("Reached target: %d\n", target);
    printf("Pos: %d; Th: %d\n", currentPoint, theta);
    markDone(target,Goals,3);

    target = selectNextObjective(Goals,3);
    if(target == -1)
        printf("All Done !\n");

    system("pause");

    return EXIT_SUCCESS;

}

