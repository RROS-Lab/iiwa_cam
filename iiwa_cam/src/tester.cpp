// use hashmap (node pointer,name,abs_path) to improve efficiency
// replace string vector with strings

#include <tree_generator.hpp>


void test1(std::vector<std::string> frame_names, std::vector<std::string> abs_paths) {
  
  // print inputs
  std::cout << "\nframe_name    :" ;
  print_string_vector(frame_names);
  std::cout << "parent_name   :" ;
  print_string_vector(abs_paths);
  
  // test generate_tree
  std::vector<cam::Frame*> frames;
  frames.resize(frame_names.size(),nullptr);
  KukaTreeNode* tree_root = generate_tree(frame_names, abs_paths, frames);
  std::cout << "generated tree:" ;
  std::vector<std::string> ans = print_tree(tree_root);
  
  // compare with original tree
  ans.erase(ans.begin());
  std::cout << std::boolalpha << (ans == frame_names) << std::endl;

  // test get_children and get_child
  std::vector<KukaTreeNode*> children = tree_root->get_children();
  KukaTreeNode* P0 = tree_root->get_child("P0");
  KukaTreeNode* P1 = tree_root->get_child("P1");

  // display results
  if(children.size()!=0) {
    std::cout << "tree_root has the following children:";
    for (std::vector<KukaTreeNode*>::iterator it = children.begin(); it != children.end(); it++) {
      std::cout << (*it)->get_name() << " ";
    }
    std::cout << std::endl;
  }
  if(P0!=nullptr) {
    std::cout << "P0's parent is:" << P0->get_parent()->get_name() << std::endl;
  }                                                                              
}

int main() {

  std::vector<std::string> frame_name {"P0","P1","P2","P3"};
  std::vector<std::string> parent_name {"","/P0","/P0","/P0"};
  test1(frame_name, parent_name);
  
  frame_name = {"P0","P1","P2","P3"};
  parent_name = {"","/P0","/P0/P1","/P0/P1/P2"};
  test1(frame_name, parent_name);
  
  frame_name = {"P0","P1","P3","P2","P3","P4","P1","P2","P3"};
  parent_name = {"","/P0","/P0/P1","/P0","/P0/P2","/P0/P2/P3","","",""};
  test1(frame_name, parent_name);

  frame_name = {};
  parent_name = {};
  test1({frame_name}, parent_name);

  frame_name = {"P0","P1","P2"};
  parent_name = {"","",""};
  test1(frame_name, parent_name);

  frame_name = {"P0","P1","P1","P2","P1","P2","P3"};
  parent_name = {"","/P0","/P0/P1","/P0/P1","","",""};
  test1(frame_name, parent_name);

  return 0;
}
