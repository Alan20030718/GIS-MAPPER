#include "m4.h"
#include "helper.h"
#include <vector>
#include "loading.h"
#include "AlgoHelper.h"
#include <cmath>

std::vector<delivery_info> delivery_status;
bool delivery_finished(const std::vector<DeliveryInf>& deliveries);
IntersectionIdx find_nearest_point(const std::vector<DeliveryInf>& deliveries, 
                                    IntersectionIdx current_location);
bool legality(int i);

std::vector<CourierSubPath> travelingCourier(
                            const std::vector<DeliveryInf>& deliveries,
                            const std::vector<IntersectionIdx>& depots,
                            const float turn_penalty){
    delivery_status.resize(deliveries.size());
    int starting_idx = 0;
    double best_distance = INFINITY;
    double current_distance;
    IntersectionIdx first_depot = 0, first_pickup = 0;
    std::vector<CourierSubPath> best_path;
    CourierSubPath segment;
    //selecting initial starting point by picking the shortest starting point
    #pragma omp parallel for
    for(int i = 0; i < depots.size(); i ++){
        for (int j = 0; j < deliveries.size(); j++){
            
            current_distance = distance(depots[i], deliveries[j].pickUp);
            if(current_distance < best_distance){
                best_distance = current_distance;
                first_depot = depots[i];
                first_pickup = deliveries[j].pickUp;
                starting_idx = j;
                
            }
        }
    }
    //first package retrieved
    delivery_status[starting_idx].retrieved = true;
    //save it into the optimized path
    segment.start_intersection = first_depot;
    segment.end_intersection = first_pickup;
    best_path.push_back(segment);  
    
    IntersectionIdx current_location, next_location;
    current_location = first_pickup;
    //continue to all delivery points
    while(!delivery_finished(deliveries)){
        next_location = find_nearest_point(deliveries, current_location);
        segment.start_intersection = current_location;
        segment.end_intersection = next_location;
        best_path.push_back(segment);  
        current_location = next_location;
    }        
    //find the nearest depot to end the journey
    best_distance = INFINITY;
    IntersectionIdx best_finish = 0;
    #pragma omp parallel for
    for(int i = 0; i < depots.size(); i ++){
            current_distance = distance(depots[i], current_location);
            if(current_distance < best_distance){
                best_distance = current_distance;
                best_finish = depots[i];
            }      
    }
    segment.start_intersection = current_location;
    segment.end_intersection = best_finish;
    best_path.push_back(segment);
    
    //calculate all path using a-star for the best path
    #pragma omp parallel for
    for(int i = 0; i < best_path.size(); i++){
       
        std::pair<IntersectionIdx, IntersectionIdx> intersection_ids = 
                    std::make_pair(best_path[i].start_intersection, best_path[i].end_intersection);
        best_path[i].subpath = findPathBetweenIntersections(intersection_ids, turn_penalty);
    }
    delivery_status.clear();
    return best_path;
}

//check if all packages have been delivered
bool delivery_finished(const std::vector<DeliveryInf>& deliveries){
    for(int i = 0; i < deliveries.size(); i++){
        if(!delivery_status[i].delivered){
            return false;
        }
    }
    return true;
}

//finds the nearest legal pickup or drop-off point
IntersectionIdx find_nearest_point(const std::vector<DeliveryInf>& deliveries, 
                                    IntersectionIdx current_location){
    int index = 0;
    bool pick_up = true;
    double current_distance;
    double best_distance = INFINITY;
    IntersectionIdx nearest = 0;
    //cant have multithreading in this loop
    //#pragma omp parallel for
    for (int i = 0; i < deliveries.size(); i++){
            
            current_distance = distance(current_location, deliveries[i].pickUp);
            if(current_distance < best_distance && !delivery_status[i].retrieved){
                best_distance = current_distance;
                nearest = deliveries[i].pickUp;
                index = i;
                pick_up = true;
            }
            current_distance = distance(current_location, deliveries[i].dropOff);
            if(current_distance < best_distance && delivery_status[i].retrieved 
               && !delivery_status[i].delivered){
                best_distance = current_distance;
                nearest = deliveries[i].dropOff;
                index = i;
                pick_up = false;
            }
    }
    if(pick_up){
        delivery_status[index].retrieved = true;
    }
    else{
        delivery_status[index].delivered = true;  
    }
    return nearest;
}

//check if drop-off is possible by checking if it has been retrieved.
bool legality(int i){
    if(delivery_status[i].retrieved){
        return true;
    }
    return false;
}
