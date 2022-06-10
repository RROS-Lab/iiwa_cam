#include <vector>
#include <string>
#include <stack>
#include <iostream>
#include "iiwa.hpp"

class KukaTreeNode {

    public: // TODO: make it private?
    std::string name; 
    std::vector<std::string> ancester_names; 
    std::vector<KukaTreeNode*> children;
    KukaTreeNode* parent;
    cam::Frame* frame;

    public: 
    KukaTreeNode(const std::string &name, std::vector<std::string> ancester_names, KukaTreeNode* parent) {
        this->name = name;
        this->ancester_names = ancester_names;
        this->parent = parent;
        this->frame = frame;
    }
};

void print_string_vector(const std::vector<std::string> &v);

// returns a string vector 
std::vector<std::string> get_ancestors(const std::string &abs_paths);
// returns a pointer to the node just been added
KukaTreeNode* insert_node(const std::string &name, const std::string &abs_path, KukaTreeNode *prev_node);
// returns a pointer to root node of the tree
KukaTreeNode* generate_tree(const std::vector<std::string> &frame_names, std::vector<std::string> &abs_paths);

std::vector<std::string> print_tree(KukaTreeNode* tree_root);
