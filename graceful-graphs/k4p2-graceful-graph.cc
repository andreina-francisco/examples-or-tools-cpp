//Copyright 2018 Maria Andreina Francisco Rodriguez (andreina@comp.nus.edu.sg)
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//http ://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.

#include "ortools/constraint_solver/constraint_solveri.h"

namespace operations_research {

	void printSolutionArray(std::vector<IntVar*> arrayOfVars);

	void gracefulGraph() {
		// Instantiate the solver.
		Solver solver("k4p2");
		const int64 numEdges = 16;

		// Create nodes
		std::vector<IntVar*> front;
		solver.MakeIntVarArray(4, 0, numEdges, "nodes", &front);

		// Create "back" nodes
		std::vector<IntVar*> back;
		solver.MakeIntVarArray(4, 0, numEdges, "Back nodes", &back);

		std::vector<IntVar*> allnodes;
		allnodes.insert(allnodes.end(), front.begin(), front.end());
		allnodes.insert(allnodes.end(), back.begin(), back.end());

		// The label of each node must be different
		solver.AddConstraint(solver.MakeAllDifferent(allnodes));

		// Edges
		std::vector<IntVar*> edges;
		solver.MakeIntVarArray(numEdges, 1, numEdges, "Edges", &edges);

		// Front
		edges[0] = solver.MakeAbs(solver.MakeSum(front[0], solver.MakeProd(front[1], -1)))->Var();
		edges[1] = solver.MakeAbs(solver.MakeSum(front[0], solver.MakeProd(front[2], -1)))->Var();
		edges[2] = solver.MakeAbs(solver.MakeSum(front[1], solver.MakeProd(front[3], -1)))->Var();
		edges[3] = solver.MakeAbs(solver.MakeSum(front[2], solver.MakeProd(front[3], -1)))->Var();

		// Front diagonals
		edges[4] = solver.MakeAbs(solver.MakeSum(front[0], solver.MakeProd(front[3], -1)))->Var();
		edges[5] = solver.MakeAbs(solver.MakeSum(front[1], solver.MakeProd(front[2], -1)))->Var();

		// Back
		edges[6] = solver.MakeAbs(solver.MakeSum(back[0], solver.MakeProd(back[1], -1)))->Var();
		edges[7] = solver.MakeAbs(solver.MakeSum(back[0], solver.MakeProd(back[2], -1)))->Var();
		edges[8] = solver.MakeAbs(solver.MakeSum(back[1], solver.MakeProd(back[3], -1)))->Var();
		edges[9] = solver.MakeAbs(solver.MakeSum(back[2], solver.MakeProd(back[3], -1)))->Var();

		// Back diagonals
		edges[10] = solver.MakeAbs(solver.MakeSum(back[0], solver.MakeProd(back[3], -1)))->Var();
		edges[11] = solver.MakeAbs(solver.MakeSum(back[1], solver.MakeProd(back[2], -1)))->Var();

		// Connecting edges
		edges[12] = solver.MakeAbs(solver.MakeSum(front[0], solver.MakeProd(back[0], -1)))->Var();
		edges[13] = solver.MakeAbs(solver.MakeSum(front[1], solver.MakeProd(back[1], -1)))->Var();
		edges[14] = solver.MakeAbs(solver.MakeSum(front[2], solver.MakeProd(back[2], -1)))->Var();
		edges[15] = solver.MakeAbs(solver.MakeSum(front[3], solver.MakeProd(back[3], -1)))->Var();



		// The labels of all the edges must be different
		solver.AddConstraint(solver.MakeAllDifferent(edges));

		//std::vector<IntVar*> allvars;

		//allvars.insert(allvars.end(), allnodes.begin(), allnodes.end());
		//allvars.insert(allvars.end(), edges.begin(), edges.end());


		// Branching heuristics
		DecisionBuilder* const db = solver.MakePhase(allnodes,
			Solver::CHOOSE_FIRST_UNBOUND,//CHOOSE_MIN_SIZE,
			Solver::ASSIGN_RANDOM_VALUE);


		// Search!
		solver.Solve(db);

		int numSolutions = 0;

		// Print
		while (solver.NextSolution()) {
			numSolutions++;
			//std::cout << "Solution " << numSolutions << " :";
			//printSolutionArray(allnodes);
		}

		const int64 elapsedTime = solver.wall_time();

		std::cout << "Total number of solutions: " << numSolutions << "\n";
		std::cout << "Total elapsed time: " << elapsedTime << " milliseconds.\n";


	} // gracefulGraph

	  // Prints the values of a vector of IntVar between square brakets.
	  // Elements of the vector are separated by white spaces.
	void printSolutionArray(std::vector<IntVar*> arrayOfVars) {
		int i = 0;
		int size = arrayOfVars.size();
		std::cout << "[ ";
		while (i < size) {
			std::cout << arrayOfVars[i]->Value() << " ";
			i++;
		}
		std::cout << "]\n";
	}

} // namespace operations_research

int main(int argc, char** argv) {
	operations_research::gracefulGraph();
	getchar();
	return 0;
} // main