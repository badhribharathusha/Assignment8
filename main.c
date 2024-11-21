#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define MAX_VERTICES 1000    // Maximum number of vertices
#define MAX_PERIODS 100      // Maximum periods

typedef struct {
    int destination;
    int cost_per_period[MAX_PERIODS];
} GraphEdge;

typedef struct {
    GraphEdge *edges;
    int edge_count;
} GraphVertex;

typedef struct {
    int vertex;
    int accumulated_cost;
    int steps_taken;
} PriorityQueueNode;

typedef struct {
    PriorityQueueNode *nodes;
    int size;
    int capacity;
} MinPriorityQueue;

GraphVertex graph[MAX_VERTICES];
int num_vertices, num_periods;
int min_cost[MAX_VERTICES][MAX_PERIODS];
int prev_vertex[MAX_VERTICES][MAX_PERIODS];
int prev_period[MAX_VERTICES][MAX_PERIODS];

// Function declarations (rearranged order)
void swap_nodes(PriorityQueueNode *a, PriorityQueueNode *b);
void heapify(MinPriorityQueue *pq, int idx);
PriorityQueueNode pop_min(MinPriorityQueue *pq);
void push_min(MinPriorityQueue *pq, PriorityQueueNode node);
void compute_shortest_paths(int start_vertex);
void print_shortest_path(int start_vertex, int end_vertex);
void process_input(FILE *file);

int main(int argc, char *argv[]) {
    if (argc != 2) {
        return 1;
    }

    FILE *file = fopen(argv[1], "r");
    if (!file) {
        return 1;
    }

    // Initialize the graph vertices
    for (int i = 0; i < num_vertices; i++) {
        graph[i].edge_count = 0;
        graph[i].edges = NULL;
    }

    process_input(file);  // Process the graph input (separated for clarity)
    fclose(file);

    int previous_start = -1;
    char input_line[256];
    while (fgets(input_line, sizeof(input_line), stdin)) {
        int start, end;
        if (sscanf(input_line, "%d %d", &start, &end) != 2) {
            continue;
        }

        if (start != previous_start) {
            previous_start = start;
            compute_shortest_paths(start);
        }

        print_shortest_path(start, end);
    }

    // Free dynamically allocated memory for graph edges
    for (int i = 0; i < num_vertices; i++) {
        free(graph[i].edges);
    }

    return 0;
}

void process_input(FILE *file) {
    int src, dest;
    while (fscanf(file, "%d %d", &src, &dest) == 2) {
        GraphEdge edge;
        edge.destination = dest;
        for (int i = 0; i < num_periods; i++) {
            fscanf(file, "%d", &edge.cost_per_period[i]);
        }
        graph[src].edge_count++;
        graph[src].edges = realloc(graph[src].edges, graph[src].edge_count * sizeof(GraphEdge));
        graph[src].edges[graph[src].edge_count - 1] = edge;
    }
}

void swap_nodes(PriorityQueueNode *a, PriorityQueueNode *b) {
    PriorityQueueNode temp = *a;
    *a = *b;
    *b = temp;
}

// Heapify function to maintain min-heap property
void heapify(MinPriorityQueue *pq, int idx) {
    int smallest = idx;
    int left_child = 2 * idx + 1;
    int right_child = 2 * idx + 2;

    if (left_child < pq->size && pq->nodes[left_child].accumulated_cost < pq->nodes[smallest].accumulated_cost)
        smallest = left_child;
    if (right_child < pq->size && pq->nodes[right_child].accumulated_cost < pq->nodes[smallest].accumulated_cost)
        smallest = right_child;

    if (smallest != idx) {
        swap_nodes(&pq->nodes[smallest], &pq->nodes[idx]);
        heapify(pq, smallest);
    }
}

// Pop minimum element from the heap
PriorityQueueNode pop_min(MinPriorityQueue *pq) {
    PriorityQueueNode min_node = pq->nodes[0];
    pq->nodes[0] = pq->nodes[pq->size - 1];
    pq->size--;
    heapify(pq, 0);
    return min_node;
}

// Insert node into priority queue and maintain min-heap
void push_min(MinPriorityQueue *pq, PriorityQueueNode node) {
    if (pq->size == pq->capacity) {
        pq->capacity *= 2;
        pq->nodes = realloc(pq->nodes, pq->capacity * sizeof(PriorityQueueNode));
    }
    pq->size++;
    int idx = pq->size - 1;
    pq->nodes[idx] = node;
    while (idx != 0 && pq->nodes[(idx - 1) / 2].accumulated_cost > pq->nodes[idx].accumulated_cost) {
        swap_nodes(&pq->nodes[idx], &pq->nodes[(idx - 1) / 2]);
        idx = (idx - 1) / 2;
    }
}

// Dijkstra's algorithm to compute the shortest paths from the start vertex
void compute_shortest_paths(int start_vertex) {
    MinPriorityQueue pq;
    pq.size = 0;
    pq.capacity = 1000;
    pq.nodes = malloc(pq.capacity * sizeof(PriorityQueueNode));

    // Initialize all distances and predecessors
    for (int i = 0; i < num_vertices; i++) {
        for (int p = 0; p < num_periods; p++) {
            min_cost[i][p] = INT_MAX;
            prev_vertex[i][p] = -1;
            prev_period[i][p] = -1;
        }
    }

    min_cost[start_vertex][0] = 0;
    push_min(&pq, (PriorityQueueNode){start_vertex, 0, 0});

    while (pq.size > 0) {
        PriorityQueueNode current_node = pop_min(&pq);
        int u = current_node.vertex;
        int current_cost = current_node.accumulated_cost;
        int steps = current_node.steps_taken;
        int period = steps % num_periods;

        if (current_cost > min_cost[u][period]) continue;

        // Traverse neighbors
        for (int i = 0; i < graph[u].edge_count; i++) {
            GraphEdge *edge = &graph[u].edges[i];
            int v = edge->destination;
            int next_steps = steps + 1;
            int next_period = next_steps % num_periods;
            int edge_cost = edge->cost_per_period[period];
            int new_cost = current_cost + edge_cost;

            if (new_cost < min_cost[v][next_period]) {
                min_cost[v][next_period] = new_cost;
                prev_vertex[v][next_period] = u;
                prev_period[v][next_period] = period;
                push_min(&pq, (PriorityQueueNode){v, new_cost, next_steps});
            }
        }
    }

    free(pq.nodes);
}

// Print the shortest path from start to end vertex
void print_shortest_path(int start_vertex, int end_vertex) {
    int shortest_distance = INT_MAX;
    int best_period = -1;
    // Find the best period for the end vertex
    for (int p = 0; p < num_periods; p++) {
        if (min_cost[end_vertex][p] < shortest_distance) {
            shortest_distance = min_cost[end_vertex][p];
            best_period = p;
        }
    }

    if (shortest_distance == INT_MAX) {
        printf("-1\n");
        return;
    }

    int path[MAX_VERTICES];
    int path_length = 0;
    int u = end_vertex;
    int p = best_period;

    // Reconstruct the shortest path
    while (u != -1) {
        path[path_length++] = u;
        int temp_u = prev_vertex[u][p];
        int temp_p = prev_period[u][p];
        u = temp_u;
        p = temp_p;
    }

    // Print the path in reverse order
    for (int i = path_length - 1; i >= 0; i--) {
        printf("%d", path[i]);
        if (i != 0) printf(" ");
    }
    printf("\n");
}
