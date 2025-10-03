#ifndef A_STAR_HPP
#define A_STAR_HPP

#include <utility>
#include <optional>
#include <vector>
#include <map>

namespace rover_utils
{

class A_star {

public:

  struct Cell {
    std::pair<int, int> coords;
    std::pair<int, int> parent_coords;

    double g_cost, h_cost;
    Cell() : coords{0, 0}, parent_coords{-1, -1}, g_cost(0.0), h_cost(0.0) {}
    Cell(std::pair<int,int> c, std::pair<int,int> p, double g, double h):coords(c), parent_coords(p), g_cost(g), h_cost(h) {}
    double f_cost() const {return g_cost + h_cost;}
  };

  struct CellComparator {
    bool operator()(const Cell& a, const Cell& b) const{
      return a.f_cost() > b.f_cost();
    }
  };

  A_star() = default;
  ~A_star() = default;
  std::optional<std::vector<std::pair<int,int>>> planPath(const std::vector<std::vector<int>> grid, const std::pair<int,int> start, const std::pair<int,int>end);

private:

  double computeHeuristics(std::pair<int,int> current, std::pair<int,int> goal);
  std::vector<std::pair<int,int>> getNeighbors(std::pair<int,int> current, const std::vector<std::vector<int>>& grid);
  std::vector<std::pair<int,int>> constructPath(Cell goal_cell, std::map<std::pair<int,int>, Cell>& all_cells);
  double distance(std::pair<int,int> coords, std::pair<int,int> neighbor_coords);

  std::map<std::pair<int, int>, double> cost_map = {
    {{1,0}, 1.0}, {{-1,0}, 1.0}, {{0,-1}, 1.0}, {{0, 1}, 1.0},
    {{1,1}, 1.414}, {{1,-1}, 1.414}, {{-1,1}, 1.414}, {{-1,-1}, 1.414}
  };
};

} // namespace rover_utils

#endif // !A_STAR_HPP
