// Copyright 2010-2017 Google
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Traveling Salesman Problem.
//
// This is a sample using the routing library to solve a Traveling Salesman
// Problem.
// The description of the problem can be found here:
// http://en.wikipedia.org/wiki/Travelling_salesman_problem.
// For small problems one can use the hamiltonian path library directly (cf
// graph/hamiltonian_path.h).
// The optimization engine finds the optimal solution.
// For larger problems, it is possible to use local search to improve solutions, first
// solutions being generated using some heuristic (e.g. cheapest addition).


#include <memory>

#include "ortools/constraint_solver/routing_flags.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/solver_parameters.pb.h"
#include "ortools/constraint_solver/routing.h"

namespace operations_research {

	//  Cost function
	int64 distance(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) {
		int matrix[13][13] = {
			{ 0, 2451, 713, 1018, 1631, 1374, 2408, 213, 2571, 875, 1420, 2145, 1972 }, // New York
		{ 2451, 0, 1745, 1524, 831, 1240, 959, 2596, 403, 1589, 1374, 357, 579 }, // Los Angeles
		{ 713, 1745, 0, 355, 920, 803, 1737, 851, 1858, 262, 940, 1453, 1260 }, // Chicago
		{ 1018, 1524, 355, 0, 700, 862, 1395, 1123, 1584, 466, 1056, 1280, 987 }, // Minneapolis
		{ 1631, 831, 920, 700, 0, 663, 1021, 1769, 949, 796, 879, 586, 371 }, // Denver
		{ 1374, 1240, 803, 862, 663, 0, 1681, 1551, 1765, 547, 225, 887, 999 }, // Dallas
		{ 2408, 959, 1737, 1395, 1021, 1681, 0, 2493, 678, 1724, 1891, 1114, 701 }, // Seattle
		{ 213, 2596, 851, 1123, 1769, 1551, 2493, 0, 2699, 1038, 1605, 2300, 2099 }, // Boston
		{ 2571, 403, 1858, 1584, 949, 1765, 678, 2699, 0, 1744, 1645, 653, 600 }, // San Francisco
		{ 875, 1589, 262, 466, 796, 547, 1724, 1038, 1744, 0, 679, 1272, 1162 }, // St.Louis
		{ 1420, 1374, 940, 1056, 879, 225, 1891, 1605, 1645, 679, 0, 1017, 1200 }, // Houston
		{ 2145, 357, 1453, 1280, 586, 887, 1114, 2300, 653, 1272, 1017, 0, 504 }, // Phoenix
		{ 1972, 579, 1260, 987, 371, 999, 701, 2099, 600, 1162, 1200, 504, 0 } }; // Salt Lake City

		return (matrix[from.value()][to.value()]);
	}
	void tsp() {
		// Let's begin with some data
		std::vector<std::string> city_names = { "New York", "Los Angeles", "Chicago", "Minneapolis", "Denver", "Dallas", "Seattle",
			"Boston", "San Francisco", "St. Louis", "Houston", "Phoenix", "Salt Lake City" };

		// TODO: how to read data from a file
		// TSPLIBReader tsp_data_reader("file-name");

		int tsp_size = city_names.size();

		int num_routes = 1; //TSP

		RoutingModel::NodeIndex depot(3);


		// Create the routing model
		if (tsp_size > 0) {
			RoutingModel routing(tsp_size, num_routes, depot);

			RoutingSearchParameters search_parameters = RoutingModel::DefaultSearchParameters();

			// Setting first solution heuristic (cheapest addition).
			search_parameters.set_first_solution_strategy(
				FirstSolutionStrategy::PATH_CHEAPEST_ARC);

			// Some local search options
			//search_parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::SIMULATED_ANNEALING);
			//search_parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
			//search_parameters.set_time_limit_ms(10000);

			search_parameters.set_use_depth_first_search(true);  // Find an optimal solution

																 //Create the distance callback, which takes two arguments(the from and to node indices)
																 //and returns the distance between these nodes.

			routing.SetCost(NewPermanentCallback(&distance));


			// Getting the solution
			const Assignment * solution = routing.SolveWithParameters(search_parameters);

			//  Solution inspection
			if (solution != NULL) {
				std::cout << "Total cost: " << solution->ObjectiveValue() << std::endl;
				for (int64 route_number = 0; route_number < num_routes; route_number++) {
					std::cout << "Route " << route_number << ": ";

					for (int64 index = routing.Start(route_number); !routing.IsEnd(index);
						index = solution->Value(routing.NextVar(index))) {
						std::cout << city_names[routing.IndexToNode(index).value()] << " -> ";
					}
					std::cout << city_names[routing.IndexToNode(routing.Start(route_number)).value()] << std::endl;
				}
			}
			else {
				std::cout << "No solution found" << std::endl;
			}
		}
	}

}  // namespace operations_research

int main(int argc, char** argv) {
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	operations_research::tsp();
	getchar();
}
