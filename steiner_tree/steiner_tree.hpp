
#ifndef STEINER_TREE_HPP
#define STEINER_TREE_HPP

/*
                input:

                        N:
                        M:
                        edges:
        x1 y1 c1
        x2 y2 c2
        ...
        xM yM cM
                        T: num of terminals
                        terminals:
        t1
        t2
        ...
        tT

                        V: the number of vertices in tree-decomposition
                        td:
                                k1 v1 v2 .. v_{k1}
                                k2 v1 v2 .. v_{k2}
                                :
                                kV v1 v2 .. v_{kV}

                        tedges:
                                x1 y1
                                x2 y2
                                :
                                x_{K-1} y_{K-1}

                output:
                        return K: the number of edges used
                        ans:
                                x1 y1
        x2 y2
        :
        x_K y_K
*/

class SteinerTree {
  public:
  SteinerTree(const std::vector<std::vector<int>>& graph, const std::vector<int>& terminals);

  private:
  int N;
  int M;
  vector<int> edges;
  int T;
  vector<int> terminals;
  int V;
  vector<int> td;
  vector<int> tedges;
  vector<int> ans;
};

#endif // STEINER_TREE_HPP
