// ia.c
// Helper functions to handle robot navigation
// & objective selection
// ---------------------------------------------------
// Martin Raynal, 2014

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
    D_180, D_MINUS_90, D_0, D_PLUS_90

} Heading;

typedef struct Waypoint {
    int start;
    int stop;    //the following node
    int dist;           //distance between nodes
    Heading orientation;

    //void* action //action to perform before move (ie, disable sonar)

} Waypoint;

//typedef struct Node {
//    int destination;    //the following node
//    int dist;           //distance between nodes
//    Heading orientation;

//    //void* action

//} Node;

//=========================================================
//  DIJKSTRA
//=========================================================
// Pompé de http://blog.developpez.com/nico-pyright/p9300/cc-win32/c_implementation_de_l_algo_de_dijkstra_e

// Playfield modelisation
int map[NB_SOMMET][NB_SOMMET] =
        //FROM
    {{ 0 , 1 ,INF,INF, 1 ,INF, 1 },
    { 1 , 0 ,INF,INF, 1 ,INF, 1 },
     {INF, 1 , 0 , 1 ,INF,INF,INF},  //T
     {INF,INF, 1 , 0 ,INF,INF,INF},  //O
     { 1 , 1 ,INF,INF, 0 , 1 , 1 },
     {INF,INF,INF,INF, 1 , 0 ,INF},
     { 1 , 1 ,INF,INF, 1 ,INF, 0 }};


void dijkstra(int sr,int ds, int path[])
{
    if (sr == ds)
        return;

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
#define COEF_PRIO 1

typedef struct Objective
{
    int baseScore;
    double successProbability;
    int wayPoint;
    int priority;

    char done;
}Objective;

/// If all goals are done or unattainable in time, returns -1
int selectNextObjective(Objective goals[], int goalCount)
{
    int retVal = -1;
    int maxScore = 0;

    int i;
    for (i=0; i<goalCount; i++)
    {
        if(goals[i].done)
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
    }

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

int main(void)
{
    printf("Start\n");

    Objective Goals[3];
    Goals[0].baseScore = 6+1; //fresque + fire
    Goals[0].successProbability = 1.0;
    Goals[0].wayPoint = FRESCO_BACK;
    Goals[0].priority = 10;
    Goals[0].done = 0;
    Goals[1].baseScore = 1; //fire
    Goals[1].successProbability = 1.0;
    Goals[1].wayPoint = FIRE;
    Goals[1].priority = 5;
    Goals[1].done = 0;
    Goals[2].baseScore = 3*6; //balls
    Goals[2].successProbability = 0.5; //to be tuned; selecting one ball over two for now
    Goals[2].wayPoint = LAUNCHPAD;
    Goals[2].priority = 1;
    Goals[2].done = 0;

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
    printf("fresco to fire\n");
    dijkstra(FRESCO_BACK, FIRE, path);
    printPath(path);

    system("pause");

    return EXIT_SUCCESS;

}

