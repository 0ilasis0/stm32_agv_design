#ifndef MAP_H
#define MAP_H

#define INF 99999
#define max_node 6
#define no_data -1

typedef struct {
    int id;
    int distance;
} CONNECTION;

typedef struct {
    int local_id;
    CONNECTION connect[8];
} LOCATION;

typedef struct {
    uint8_t current_count;
    int8_t direction[max_node];
    uint16_t address_id[max_node];
} MAP_DATA;

extern MAP_DATA map_current_data;

void map_setup(void);
void map_init(void);
void floyd_warshall(void);
void build_current_map_data(int from, int to);
int get_index_by_id(int id);


#endif
