'''pastpath.route : functions associated with creating routes
'''
from flask import current_app as app
from openrouteservice import client, directions, distance_matrix, places
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from random import shuffle

def calc_distance_matrix(marker_coords):
    ors_clnt = client.Client(key=app.config['ORS_KEY'])

    request = {'locations': marker_coords,
           'profile': 'foot-walking',
           'metrics': ['duration']}
    
    dist_matrix = ors_clnt.distance_matrix(**request)
    return dist_matrix

def getDistance(from_id, to_id, dist_matrix):
    return int(dist_matrix['durations'][from_id][to_id])

def optimal_route_from_matrix(marker_coords, marker_names, dist_matrix, start=0):
    tsp_size = len(marker_names)
    num_routes = 1

    optimal_coords = []
    marker_order = []
    route_str = ''

    if tsp_size > 0:
        routing = pywrapcp.RoutingModel(tsp_size, num_routes, start)
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

        # Create the distance callback, which takes two arguments (the from and to node indices)
        # and returns the distance between these nodes.
        dist_callback = lambda from_id, to_id: getDistance(from_id, to_id, dist_matrix)
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
        # Solve, returns a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            index = routing.Start(start) # Index of the variable for the starting node.

            # construct optimal_coords (coords) and route (names)
            # by looping through nodes in solution
            for node in range(routing.nodes()):
                marker_order.append(routing.IndexToNode(index))
                optimal_coords.append(marker_coords[routing.IndexToNode(index)])
                route_str += str(marker_names[routing.IndexToNode(index)]) + ' -> '
                index = assignment.Value(routing.NextVar(index))
            route_str += str(marker_names[routing.IndexToNode(index)])
            optimal_coords.append(marker_coords[routing.IndexToNode(index)])
            
    return optimal_coords, marker_order, route_str

def directions_route_duration(route_coords):
    ors_clnt = client.Client(key=app.config['ORS_KEY'])
    request = {'coordinates': route_coords,
               'profile': 'foot-walking',
               'geometry': 'true',
               'format_out': 'geojson',
              }

    route = ors_clnt.directions(**request)
    # duration in minutes
    duration = route['features'][0]['properties']['summary']['duration'] / 60

    return route, duration

def compare_random_optimized(optimal_coords):
    # NB: assumes not returning to starting point in either case
    # See what a random tour would have been
    tour_start = optimal_coords[0]
    marker_coords_shuffle = optimal_coords[1:].copy()
    shuffle(marker_coords_shuffle)
    marker_coords_shuffle_loop = [tour_start] + marker_coords_shuffle
    request = {'coordinates': marker_coords_shuffle_loop,
               'profile': 'foot-walking',
               'geometry': 'true',
               'format_out': 'geojson',
              }
    random_route, random_duration = directions_route_duration(marker_coords_shuffle_loop)

    optimal_route, optimal_duration = directions_route_duration(optimal_coords)

    return {'random_route': random_route,
            'random_duration': random_duration,
            'optimal_route': optimal_route,
            'optimal_duration': optimal_duration}
