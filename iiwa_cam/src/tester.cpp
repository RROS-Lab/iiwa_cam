// use hashmap (node pointer,name,abs_path) to improve efficiency
// replace string vector with strings
// makefile for debugging purpose

#include "tree_generator.h"

void test(std::vector<std::string> frame_names, std::vector<std::string> abs_paths) {
  
  // print inputs
  std::cout << "\nframe_name    :" ;
  print_string_vector(frame_names);
  std::cout << "parent_name   :" ;
  print_string_vector(abs_paths);
  
  // generate outputs
  KukaTreeNode* tree_root = generate_tree(frame_names, abs_paths);
  std::cout << "generated tree:" ;
  std::vector<std::string> ans = print_tree(tree_root);

  // compare 
  ans.erase(ans.begin());
  std::cout << std::boolalpha << (ans == frame_names) << std::endl;
}

int main() {

  std::vector<std::string> frame_name {"P0","P1","P2","P3"};
  std::vector<std::string> parent_name {"","/P0","/P0/P1","/P0/P1/P2"};
  test(frame_name, parent_name);

  frame_name = {"P0","P1","P3","P2","P3","P4","P1","P2","P3"};
  parent_name = {"","/P0","/P0/P1","/P0","/P0/P2","/P0/P2/P3","","",""};
  test(frame_name, parent_name);

  frame_name = {};
  parent_name = {};
  test(frame_name, parent_name);

  frame_name = {"P0","P1","P2"};
  parent_name = {"","",""};
  test(frame_name, parent_name);

  frame_name = {"P0","P1","P1","P2","P1","P2","P3"};
  parent_name = {"","/P0","/P0/P1","/P0/P1","","",""};
  test(frame_name, parent_name);

  return 0;
}
