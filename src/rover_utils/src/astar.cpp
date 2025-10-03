#include "rover_utils/astar.hpp"
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>

namespace rover_utils
{

std::optional<std::vector<std::pair<int,int>>> A_star::planPath(const std::vector<std::vector<int>> grid, const std::pair<int,int> start, const std::pair<int,int> goal)
{
  // Define Variables
  std::priority_queue<A_star::Cell, std::vector<A_star::Cell>, A_star::CellComparator> open_set;
  std::set<std::pair<int,int>> closed_set;
  std::map<std::pair<int,int>, Cell> all_cells;

  // Add start position to the open set
  A_star::Cell start_cell(start, {-1,-1}, 0.0, computeHeuristics(start, goal));
  open_set.push(start_cell);
  all_cells[start] = start_cell;

  while(!open_set.empty()){

    // Get cell with lowest f_cost from open_set
    auto current_cell = open_set.top();
    open_set.pop();

    // If current cell is already in the close set dont process it again
    if (closed_set.find(current_cell.coords) != closed_set.end()) {
        continue;
    }

    if (current_cell.coords == goal){
      //  If it's the goal, reconstruct path and return
      return constructPath(current_cell, all_cells);

    } else {
        // Move cell from open_set to closed_set
        closed_set.insert(current_cell.coords);

        auto neighbors = getNeighbors(current_cell.coords, grid);
        for (auto neighbor : neighbors){

          // Check if we already processed the neighboing cell
          if (closed_set.find(neighbor)!= closed_set.end()) {
            // We already processed this cell, do not revisit
            continue;
          }
          // Havent processed this cell yet, but can be in the open set
          double g_cost = current_cell.g_cost + distance(current_cell.coords, neighbor);

          // Add this new cell if it wasnt processed before or the cost is better
          if(all_cells.find(neighbor) == all_cells.end() || g_cost < all_cells[neighbor].g_cost){

            A_star::Cell new_cell(neighbor, current_cell.coords, g_cost, computeHeuristics(neighbor, goal));
            open_set.push(new_cell);
            all_cells[neighbor] = new_cell;
          }
        }

      }

    }

  return std::nullopt;
}

double A_star::computeHeuristics(std::pair<int,int> current, std::pair<int,int> goal){

  int h_x = goal.first - current.first;
  int h_y = goal.second - current.second;

  // Simple Heuristic, Euclidian Distance
  double heuristic = std::sqrt(h_x * h_x + h_y * h_y);

  return heuristic;
}

std::vector<std::pair<int,int>> A_star::getNeighbors(std::pair<int,int> current, const std::vector<std::vector<int>>& grid){

  // Compute possible neighboring grids for current
  std::vector<std::pair<int,int>> neighbors;

  std::vector<std::pair<int, int>> possible_moves = {
    {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {-1,1}, {1,-1}, {-1,-1}
  };

  for(auto move : possible_moves){

    int x_idx = current.first + move.first;
    int y_idx = current.second + move.second;

    if(x_idx >= 0 && x_idx < grid[0].size() && y_idx >= 0 && y_idx < grid.size() && grid[y_idx][x_idx] == 0){
      neighbors.push_back({x_idx, y_idx});
    }
  }
  return neighbors;
}

std::vector<std::pair<int,int>> A_star::constructPath(Cell goal_cell, std::map<std::pair<int,int>, Cell>& all_cells){

  std::vector<std::pair<int,int>> path;
  A_star::Cell current = goal_cell;
  while(current.parent_coords != std::make_pair(-1, -1)){

    path.push_back(current.coords);
    current = all_cells[current.parent_coords];

  }
  // adding the start and reversing order
  path.push_back(current.coords);
  std::reverse(path.begin(), path.end());
  return path;
}

double A_star::distance(std::pair<int,int> coords, std::pair<int,int> neighbor_coords){
  // Use member variable cost_map
  auto pair = std::make_pair(neighbor_coords.first - coords.first, neighbor_coords.second - coords.second);
  return cost_map[pair];
}

} // namespace rover_utils
