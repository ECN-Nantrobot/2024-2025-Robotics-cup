#ifndef A_STAR_H
#define A_STAR_H

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <position.h>
#include <queue>
#include <vector>

namespace ecn
{

inline void saveAstarPathToFile(const std::string& filename, const std::vector<Position>& path)
{
    std::ofstream outFile(filename);
    if (!outFile)
    {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    for (const auto& pos : path)
    {
        outFile << pos.x << " " << pos.y << "\n";
    }

    outFile.close();

    std::cout << "A*path saved to file: " << filename << std::endl;
}

std::vector<Position> loadPathFromFile(const std::string& filename)
{
    std::cout << "Loading A*path from file: " << filename << " ... ";
    std::ifstream inFile(filename);
    if (!inFile) {throw std::runtime_error("Error: Could not open file " + filename);}

    // Count the number of lines to determine the size of the vector
    size_t lineCount = 0;
    std::string line;
    while (std::getline(inFile, line))
    {
        ++lineCount;
    }

    // Reset the file read position
    inFile.clear();
    inFile.seekg(0, std::ios::beg);

    // Preallocate the vector with the determined size
    std::vector<Position> path(lineCount);

    // Read positions into the vector
    Point pos;
    for (size_t i = 0; i < lineCount && inFile >> pos.x >> pos.y; ++i)
    {
        path[i] = pos;
    }

    std::cout << "ok" << std::endl;
    return path;
}


// anonymous namespace for implementation details
namespace astar_impl
{
// resulting tree + owner of objects

template <class Node> class Tree
{
    struct NodeWithParent
    {
        Node node;
        size_t parent;

        explicit NodeWithParent(Node&& node, size_t parent) : node{ std::forward<Node>(node) }, parent{ parent } {}
        // tell vector<NodeWithParent> to use move
        NodeWithParent(NodeWithParent&& rhs) noexcept = default;
    };

    std::vector<NodeWithParent> nodes;
    std::vector<size_t> closedSet;

public:
    /// a Tree takes ownership of the nodes it is given
    inline size_t insert(Node&& node, size_t parent)
    {
        nodes.emplace_back(std::forward<Node>(node), parent);
        return nodes.size() - 1;
    }

    inline Node& operator()(size_t idx) { return nodes[idx].node; }

    inline size_t parent(size_t idx) { return nodes[idx].parent; }

    inline void setParent(size_t node, size_t parent) { nodes[node].parent = parent; }

    inline void close(size_t idx) { closedSet.push_back(idx); }

    inline bool isVisited(const Node& node) const
    {
        return std::find_if(closedSet.rbegin(), closedSet.rend(), [&](const auto idx) { return nodes[idx].node == node; }) != closedSet.rend();
    }

    std::vector<Node> fullPathTo(size_t best, int dist) const
    {
        std::vector<Node> path;
        // build list from end to start
        while (best)
        {
            path.push_back(nodes[best].node);
            best = nodes[best].parent;
        }
        path.push_back(nodes[0].node);
        // list from start to end
        std::reverse(path.begin(), path.end());
        for (size_t i = 1; i < path.size(); ++i)
            path[i - 1].print(path[i]);

        // std::cout << "solved in " << path.size() - 1 << " steps, distance is " << dist << std::endl;

        return path;
    }
};

// Generic node with distance to start (g) + total cost from heuristic (f = g+h)
template <class Node> struct NodeWithCost
{
    size_t node;
    double f;
    double g;
    inline NodeWithCost(size_t node, double h = 0, double g = 0) : node{ node }, f{ h + g }, g{ g } {}
    bool operator<(const NodeWithCost& other) const
    {
        // if(f == other.f)
        //   return g < other.g;
        return f >= other.f;
    }
};

// Priory queue with find
template <class Node> class Queue : public std::priority_queue<NodeWithCost<Node>, std::vector<NodeWithCost<Node>>>
{
    Tree<Node>& tree;

public:
    explicit Queue(Tree<Node>& tree) : tree{ tree } {}

    std::optional<NodeWithCost<Node>> find(const Node& node)
    {
        const auto same{ std::find_if(this->c.rbegin(), this->c.rend(), [&](const auto& twin) { return tree(twin.node) == node; }) };
        if (same != this->c.rend())
            return *same;
        return {};
    }
};
} // namespace astar_impl


// templated version of A* algorithm
template <class Node> std::vector<Node> Astar(Node start, Node goal)
{
    bool show = false; // if we should display during A*

    auto t0 = std::chrono::system_clock::now();

    // keep track of who comes from who
    astar_impl::Tree<Node> tree;
    tree.insert(std::move(start), 0);

    astar_impl::Queue<Node> queue(tree);
    queue.push({ 0, start.heuristic(goal), 0 });
    if (show)
        start.start();

    int evaluated = 0, created = 0, shortcut = 0;
    evaluated++;
    while (!queue.empty())
    {
        const auto best = queue.top();

        if (tree(best.node) == goal)
        {
            // std::cout << created << " nodes created, " << evaluated << " evaluated, " << shortcut << " shortcuts found" << std::endl;
            // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t0).count() << " ms" << std::endl;

            return tree.fullPathTo(best.node, best.g);
        }

        tree.close(best.node);
        queue.pop();
        if (show)
        {
            auto parent = tree.parent(best.node);
            if (parent)
                tree(best.node).show(true, tree(parent));
        }

        //CHANGED CHANGED CHANGED
        // auto children{ tree(best.node).children() };

        auto children = tree(best.node).children(tree(tree.parent(best.node)));
        created += children.size();


        // // Sort children by heuristic: not useful
        // std::sort(children.begin(), children.end(), [&](const Position& a, const Position& b) { return a.heuristic(goal) < b.heuristic(goal); });

        // // to avoid equal costs leading to favorite directions
        // std::random_shuffle(children.begin(), children.end());

        for (auto&& child : children)
        {
            // ensure we have not been here
            if (tree.isVisited(child))
                continue;

            const auto child_g = best.g + child.distToParent();
            if (const auto& twin{ queue.find(child) }; !twin)
            {
                const auto idx{ tree.insert(std::move(child), best.node) };
                const auto child_in_tree{ tree(idx) };
                queue.push({ idx, child_in_tree.heuristic(goal) + child_g, child_g });
                evaluated++;
                if (show)
                    child.show(false, tree(best.node));
            }
            else if (twin->g > child_g)
            {
                tree.setParent(twin->node, best.node);
                queue.push({ twin->node, twin->f - twin->g + child_g, child_g });
                shortcut++;
            }
        }
    }

    // while loop exit means no solutions
    std::cout << "No solutions " << std::endl;
    std::cout << created << " nodes created, " << evaluated << " evaluated, " << shortcut << " shortcuts found" << std::endl;
    return {};
}

} // namespace ecn

#endif // A_STAR_H
