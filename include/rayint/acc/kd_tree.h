/*
 * Copyright (C) 2015, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef ACC_KDTREE_HEADER
#define ACC_KDTREE_HEADER

#include <queue>
#include <stack>
#include <limits>

#include <math/vector.h>

ACC_NAMESPACE_BEGIN

bool compare(std::pair<std::size_t, float> a, std::pair<std::size_t, float> b) {
    return a.second < b.second;
}

template <uint16_t K>
class KDTree {
private:
    std::vector<math::Vector<float, K> > const & points;
    struct Node {
        decltype(K) d;
        std::size_t first;
        std::size_t last;
        std::size_t id;
        Node* left;
        Node* right;
        Node(decltype(K) d, std::size_t first, std::size_t last)
            : d(d), first(first), last(last), left(nullptr), right(nullptr) {}
        bool is_leaf() const {return left == nullptr && right == nullptr;}
    };

    Node *root;

public:
    ~KDTree();

    KDTree(std::vector<math::Vector<float, K> > const & points);

    std::pair<std::size_t, float>
    find_nn(math::Vector<float, K> point) const;

    std::vector<std::pair<std::size_t, float> >
    find_nns(math::Vector<float, K> point, std::size_t n) const;
};

KDTree::~KDTree() {
    std::queue<Node*> q;
    q.push(root);
    while (!q.empty()) {
        Node *node = q.front(); q.pop();
        if (node == nullptr) continue;

        q.push(node->left);
        q.push(node->right);
        delete node;
    }
};

KDTree::KDTree(std::vector<math::Vector<float, K> > const & points)
    : points(points) {
    std::vector<std::size_t> indices(points.size());
    for (std::size_t i = 0; i < indices.size(); ++i)
        indices[i] = i;
    root = new Node(0, 0, indices.size());

    std::queue<Node*> q;
    q.push(root);
    while (!q.empty()) {
        Node *node = q.front(); q.pop();
        decltype(K) d = node->d;
        std::sort(&indices[node->first], &indices[node->last],
            [&points, d] (std::size_t a, std::size_t b) -> bool {
                return points[a][d] < points[b][d];
            }
        );
        d = (d + 1) % K;
        std::size_t mid = (node->last + node->first) / 2;
        node->id = indices[mid];
        if (mid - node->first > 0) {
            node->left = new Node(d, node->first, mid);
            q.push(node->left);
        }
        if (node->last - (mid + 1) > 0) {
            node->right = new Node(d, mid + 1, node->last);
            q.push(node->right);
        }
    }
}

std::pair<std::size_t, float>
KDTree::find_nn(math::Vector<float, K> point) const {
    return find_nns(point, 1)[0];
}

std::vector<std::pair<std::size_t, float> >
KDTree::find_nns(math::Vector<float, K> point, std::size_t n) const {
    float sdist = std::numeric_limits<float>::max();
    std::pair<std::size_t, float> nn = std::make_pair(-1, sdist);
    std::vector<std::pair<std::size_t, float> > nns(n, nn);

    std::stack<std::pair<Node const*, bool> > s;
    s.push(std::make_pair(root, true));
    while (!s.empty()) {
        Node const *node = s.top().first;
        bool down = s.top().second;
        s.pop();
        if (node == nullptr) continue;

        float diff = point[node->d] - points[node->id][node->d];
        if (down) {
            float dist = (point - points[node->id]).norm();
            if (dist < sdist) {
                nn = std::make_pair(node->id, dist);
                nns.push_back(nn);
                std::sort(nns.begin(), nns.end(), compare);
                nns.pop_back();
                sdist = nns.back().second;
            }

            if (node->is_leaf()) continue;

            s.push(std::make_pair(node, false));
            if (diff < 0.0f) {
                s.push(std::make_pair(node->left, true));
            } else {
                s.push(std::make_pair(node->right, true));
            }
        } else {
            if (std::abs(diff) >= sdist) continue;

            if (diff < 0.0f) {
                s.push(std::make_pair(node->right, true));
            } else {
                s.push(std::make_pair(node->left, true));
            }
        }
    }
    return nns;
}

ACC_NAMESPACE_END

#endif /* ACC_KDTREE_HEADER */
