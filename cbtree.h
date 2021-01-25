
//---------------------------------------------------------------------------
#ifndef btreeH
#define btreeH
//---------------------------------------------------------------------------
#include <vector>
#include <cassert>
#include "fplan.h"

//---------------------------------------------------------------------------
typedef bool DIR;
const bool LEFT=0, RIGHT=1;
const int vector_max_size = 10000;  // it is used allocation enough memory.

//---------------------------------------------------------------------------

/* CB-Tree */
// CB-Tree is extended from the FPlan class. CB_Tree doesn't main basic variables. It has functions.
// We can make customized functions in here.
// And We create the constructor of CB_Tree, but it has member vairiables of its parent FPlan Class.
// The aim of this is to get the private members in the external of the Class and easily add the customize functions and easily understand.
// We can understand the basic structure of the CB-Tree from the Fplan Class.
// And next we can understand the actions in the CB_Tree.
class CB_Tree : public FPlan{
public:
    CB_Tree(float calpha=1) :FPlan(calpha) { prev_tree = nullptr; }

    //-----------------[CB-Tree optioins we don't use these functions in WB-Tree]----------------------------
    virtual void init();
    virtual void packing();
    virtual void perturb();
    //---------------------------------------------

    void          init_with_node_indices(vector<int> nodes);    // init with nodes. it doesn't have nodes. so set the nodes and init.
    void          init_with_out_node();                         // init without nodes. it has nodes already.
    void          make_tiles();                                 // create the initial tiles for each module. tiles map are created.
    void          init_tiles();                                 // init tiles as Nil.

//-----------------------------------------------------------------------------------------------------    
    void          insert_node_by_id(Node* parent, int moduleId);    // insert the node.
    void          copy_tree(vector<Node> &tree);                    // copy the current CB-Tree
    int           take_node_random();                              //remove the random node and return that node.
    bool          take_node(int mod_id);                           // remove the specific node and it doesn't exist, returns false.
    void          swap_node_by_id (int m1, int m2);                // swap the nodes.
    Node*         find_node_by_id(int moduleID);                   // find node with id
    Node*         find_node_random();                              // find random node.

//--------------------------------------------------------------------------------------------------------
    void          show_tree();                                     // show the tree result in the console.
    vector<Node*> allnodes();                                      // get All nodes of the CB-Tree.
    double           getDensityArea();                             // calculate the sum area of the layout patterns. [not used in WB-Tree]

    void          destroy();                                       // delete the CB-Tree

    map<int,Tiles> tile_map;                                        // Tiles Map, tiles are mapped with the modules's id as the key
    vector<int>     inds;                                           // nodes array. it stores the modules' ids.
    int             estimate_width,estimate_height;                 // CB-Tree's region info.

protected:

    void          place_module(Node* mod);                          // place modules using corner-stitch algorithm. useing tiles.
    void          correct_location_by_preplaced(int &x, int &y, Node* mod);     // process the preplaced constraints
    void          update_tiles(Node* mod, int pid, int contour = NIL);          // After one module placed, Tile map is modified, it proceeds that modifcation.
    int           calc_newY(int Dbot, int Dfin, int curY);          // proceed the Fin-alignment constraint.
    void          place_patterns(Node* mod);                        // place the patterns in the modules.
    void          clear();                                          // initialize the CB-Tree [not used in WB-Tree].

//------------------------------------------------CB-Tree's base options [no used in WB-Tree]------------------------------------------
    void          swap_node(Node *n1, Node *n2);                    // swap_node in CB-Tree. [not used in WB-Tree]
    void          insert_node(Node *parent,Node *node);             // insert_node in CB-Tree. doesn't insert new node.  [not used in WB-Tree]
    void          delete_node(Node *node);                          // remove the node from the tree in CB-Tree. doesn't delete it.  [not used in WB-Tree]
// ----------------------------------------------------------------------------------------------------------------------

    void          calc_total_area();                                // calculate the sum of the modules's area. [not used in WB-Tree]
private:

    vector<Node>* prev_tree;                                        // keep the previous solution's tree, [not used in WB-Tree]
};

//---------------------------------------------------------------------------
#endif
