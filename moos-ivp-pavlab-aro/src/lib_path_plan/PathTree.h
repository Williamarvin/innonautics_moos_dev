#ifndef GRAPHNODE_HEADER
#define GRAPHNODE_HEADER


#include <string>
#include <vector>
#include <map>

class PathTree
{

public:
  PathTree(){};
  ~PathTree(){};

  std::map<std::string, double> getRewards() {return m_rewards;};
  std::vector<std::pair<std::string,std::string> > getEdges() {return m_edges;};
  std::string getRootState() {return m_root_state;};
  int getIterations() {return m_iterations;};

  void setRewards(std::map<std::string, double> rewards) {m_rewards = rewards;};
  void setEdges(std::vector<std::pair<std::string,std::string> > edges) {m_edges = edges;};
  void setRootState(std::string root_state) {m_root_state = root_state;};
  void setIterations(int iterations) {m_iterations = iterations;};

  void addEdge(std::pair<std::string,std::string> edge) {m_edges.push_back(edge);};
  void addReward(std::string state, double reward) {m_rewards[state] = reward;}

protected:
  std::map<std::string, double> m_rewards;
  std::vector<std::pair<std::string,std::string> > m_edges;
  std::string m_root_state;
  int m_iterations;
};

#endif
