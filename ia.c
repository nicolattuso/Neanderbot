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
    int destination;    //the following node
    int dist;           //distance between nodes
    Heading orientation;

    //void* action

} Waypoint;

typedef struct Node {
    int destination;    //the following node
    int dist;           //distance between nodes
    Heading orientation;

    //void* action

} Node;

//=========================================================
//  DIJKSTRA
//=========================================================

// Playfield modelisation
int map[NB_SOMMET][NB_SOMMET] =
    {{ 0 , 1 ,INF,INF, 1 ,INF, 1 },
     { 1 , 0 , 1 ,INF, 1 ,INF, 1 },
     {INF,INF, 0 , 1 ,INF,INF,INF},
     {INF, 1 ,INF, 0 ,INF,INF,INF},
     { 1 , 1 ,INF,INF, 0 , 1 , 1 },
     {INF,INF,INF,INF, 1 , 0 ,INF},
     { 1 , 1 ,INF,INF, 1 ,INF, 0 }};


void dijkstra(int sr,int ds, int path[])
{
    if (sr == ds)
        return;

    typedef enum Label
    {
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
    int path[50];
    int z;
    for (z = 0 ; z < 50 ; z++)
        path[z] = -1;

    printf("Start to fresco\n");
    dijkstra(STARTPOINT, FRESCO_BACK, path);
    printPath(path);
    printf("Fire to fresco");
    dijkstra(FIRE, FRESCO_BACK, path);
    printPath(path);
    printf("Lauch point to fresco");
    dijkstra(LAUNCHPAD, FRESCO_BACK, path);
    printPath(path);
    printf("fresco to fire");
    dijkstra(FRESCO_BACK, FIRE, path);
    printPath(path);

    system("pause");

    return EXIT_SUCCESS;

}

