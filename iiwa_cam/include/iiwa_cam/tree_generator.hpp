#include <vector>
#include <string>
#include <stack>
#include <iostream>
#include <iiwa.hpp>

class KukaTreeNode {

    public: // TODO: make it private?
    std::string name; 
    std::vector<std::string> ancester_names; 
    std::vector<KukaTreeNode*> children;
    KukaTreeNode* parent;
    cam::Frame* frame;

    public: 
    KukaTreeNode(const std::string &name, std::vector<std::string> ancester_names, KukaTreeNode* parent, cam::Frame* frame) {
        this->name = name;
        this->ancester_names = ancester_names;
        this->parent = parent;
        this->frame = frame;
    }

    // convert a svectortring to a string vector
    std::vector<std::string> get_ancestors(const std::string &abs_paths);
    // returns a pointer to the node just been added
    KukaTreeNode* insert_node(const std::string &name, const std::string &abs_path, KukaTreeNode *prev_node, cam::Frame* frame);

    // get child
    KukaTreeNode& get_child(const std::string childname) {
        for (std::vector<KukaTreeNode*>::iterator it = this->children.begin(); it != this->children.end(); it++) {
            if ((*it)->name==childname) {
                // KukaTreeNode *&ref = *it;
                return **it;
            }
        }
        
        // TO ASK: handle to handle this situation
        std::cout << childname << " is not a child of " << this->name << std::endl;
        // KukaTreeNode *&ref = this;
        return *this;
    }

    // get children
    std::vector<KukaTreeNode*> get_children() {
        // TO CHECK: return type is vector<pointer>
        return this->children;
    }
};


void print_string_vector(const std::vector<std::string> &v) {
    if (v.size()==0) {
        std::cout << "empty";
    }
    for (int i = 0; i < v.size(); i++) {
        std::cout << "[" << v[i] << "]";
    }
    std::cout << std::endl;
}

std::vector<std::string> get_ancestors(const std::string &abs_paths) {

    std::vector<std::string> ancester_names;
    std::string delim = "/";
    size_t start_loc = 0;
    size_t end_loc = 0;

    while(abs_paths.find(delim, start_loc)==0){
        start_loc++;
    }
    while (end_loc!=std::string::npos) {
        end_loc = abs_paths.find(delim, start_loc);
        ancester_names.push_back(abs_paths.substr(start_loc, end_loc-start_loc));
        start_loc = end_loc+1;
    }
    return ancester_names;
}

KukaTreeNode* insert_node(const std::string &name, const std::string &abs_path, KukaTreeNode *prev_node, cam::Frame* frame) {

        // compare abs_path of the new node with abs_path of the prev_node
        // if matches -> set prev_node to be the new node's parent node
        // otherwise -> check if path of the earlier node matches the path
        // once matches, set this node to be the new node's parent node
        
        KukaTreeNode* new_node;
        KukaTreeNode* parent_node = prev_node;
        bool found = false;

        std::vector<std::string> ancester_names = get_ancestors(abs_path);
        ancester_names.pop_back();

        while (!found) {
            if (ancester_names == parent_node->ancester_names) {
                found = true;                    
                break;
            }
            parent_node = parent_node->parent;
        }
        
        new_node = new KukaTreeNode(name,get_ancestors(abs_path), parent_node, frame);
        parent_node->children.push_back(new_node); 

        return new_node;
    }

// preorder trasversal

std::vector<std::string> print_tree(KukaTreeNode* tree_root) {

    std::stack<KukaTreeNode*> s;
    std::vector<std::string> ans;
    s.push(tree_root);

    while (!s.empty()) {
        KukaTreeNode* temp = s.top();
        s.pop();
        ans.push_back(temp->name);
        for (int i = temp->children.size()-1; i >= 0; i--) {
            s.push(temp->children[i]);
        }
    }
    print_string_vector(ans);
    return ans;
}

KukaTreeNode* generate_tree(const std::vector<std::string> &frame_names, std::vector<std::string> &abs_paths, std::vector<cam::Frame*> &frames) {
    
    // add /r to front of all elements in parent_naparent_naparent_naparent_naparent_naparent_na
    std::vector<KukaTreeNode*> children;
    KukaTreeNode* parent;
    cam::Frame* frame;
    for(std::vector<std::string>::size_type i = 0; i != abs_paths.size(); i++) {
        abs_paths[i] = "/r" + abs_paths[i];
    }

    // initialize root node
    std::vector<std::string> empty;
    KukaTreeNode* root = new KukaTreeNode("r", empty, nullptr, nullptr);
    KukaTreeNode* prev_node = root;

    // create tree
    for(std::vector<std::string>::size_type i = 0; i != frame_names.size(); i++) {
        prev_node = insert_node(frame_names[i], abs_paths[i], prev_node, frames[i]);
    }
    return root;
}