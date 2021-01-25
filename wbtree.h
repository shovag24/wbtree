#ifndef wbtreeH
#define wbtreeH

#include "cbtree.h"
#include <stdlib.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <climits>
#include "math.h"


/* 
    The mesh node of the WB-Tree. It means the winow mesh. 
    Each window could have CB-Tree as a child or not.
*/
struct WBNode
{
    int         row,col;    // window node's row and column number. These values are used for self-aligned packing.
    CB_Tree*    cbtree;     // CB-Tree Class variable pointer. It means the cb-Tree. if there is no CB-Tree in the window node, this value is nullptr.
};

/*
    WB_Tree class is for WB-Tree implementation.
*/
class WB_Tree
{

public:

//[1]. Initialize Stage. 
    
    /* 
        Initialize the WB-Tree. It parses the input files and stores them, processes the PreProcessing stage. 
        The params alpha, filename, times, local, term_temp are made from the run command.
    */
    void init(float alpha, char *filename, int times, int local, float term_temp);

    /*
        The init_cbtree function. It initialized the CB-Tree.
        Set the module, create network, layout patterns and constraints in CBTree.
    */
    void init_cbtree(CB_Tree &fp);

    /*
        Read the modules' information from the input file [apte].
        It stores them to the private member variable [modules]
    */
    void read_module_info();

    /*
        Read the Fin-Fet design constraints and the Layout patterns' information from the input file [apte.design].
    */
    void read_design_rules();

    /*
        Set the modules's initial position.
        ex: In the apte file, module's position, -300 -300 800 400, it is corrected from read_dimension as 0 0 1100 700.
    */
    void read_dimension(Module &mod);

    /*
        Get the pin information from the input file [apte].
        Those informations are used for creating network and calculating Wirelength.
    */
    void read_IO_list(Module &mod,bool parent);

    /*
        Get the relation information between the modules from the input file [apte].
        Those informations are used for creating network and calculating Wirelength.
    */
    void read_network();

    /*
        It creates the Network between the modules.
        It is used for calculating Wirelength.
    */
    void create_network();

    /*
        Read the geometrical constraints from the input file [apte.constraint].
        It stores the constraints data to the private member variable, constraints.
    */
    void read_constraint();

    /*
        Read the Proximity constraints from the input file and store these into the member variable [constraints.proximiy]
    */
    void read_proximity_constraint();

    /*
        Read the Fixed boundary constraints from the input file and store these into the member variable [constraints.fixed_boundary]
    */
    void read_fixed_boundary_constraint();

    /*
        Read the Boundary constraints from the input file and store these into the member variable [constraints.boundary]
    */
    void read_boundary_constraint();

    /*
        Read the Minimum_Separation constraints from the input file and store these into the member variable [constraints.minimum_separation]
    */
    void read_minimum_seperation();

    /*
        Read the Symmetry constraints from the input file and store these into the member variable [constraints.symmetry]
    */
    void read_symmetry_constraint();

    /*
        Read the Preplaced constraints from the input file and store these into the member variable [constraints.preplaced]
    */
    void read_preplaced_constraints();

//[2]. Main Processing.

    /*
        Assign the mask number using greedy method.
        ex] we traverse all the layout patterns in every module and set the mask ids as 0,1,2,3... to the mask numbers, and repeat from 0.
            These are initialize values and they don't affect the final result.
    */
    void mask_assignment();

    /*
        Process the preprocessing stage.
        Extract the critical layout patterns and construct the wb-tree.
    */
    void preprocessing();

    /*
        Extract the critical layout patterns
        We traverse all the layout patterns, separate the patterns to internal and external.
        ex] If the minimum_distance(for mask conflict) is 300, the patterns that are placed in distanced 300 region from boundary are external, others are internal.
    */
    void extract_critical_layout_patterns();

    /*
        Construct the WB-Tree.
        We create a CB-Tree, and connect all modules into it. 
        And next we choose the random window mesh node, and connect the CB-Tree into it, finish construction.
    */
    void construct_wb_tree();

    /*
        Do perturb [t] times and calculate the average cost for getting good result.
        normalize cost gets the average cost before the Floorplanning and We start the floorplan based on this average cost.
    */
    void normalize_cost(int t);

    /* 
        Perturbation. Do perturb and self-aligned packing. 
        In the perturbation function, we perturb the modules according to the options Op1~Op6.
        And check the infeasible solution using look_ahead_density_checking and geometrical_constraint_checking to avoid infeasible solutions.
        If it is feasible solution, we do self_aligned_packing and check the global/local_mask_density.
        After that, we calculate the cost. and finish the perturbation.
    */
    void perturbation();
    /*
        For normalize cost, we made it specially. It does only main perturb actions Op1~Op6.
    */
    void perturb();
    
    /* Perturb Operations. */
    void Op1(); // Move a module into another Window mesh's CB-Tree.
    void Op2(); // Swap 2 modules between 2 windows' CB-Trees.
    void Op3(); // Move a module into the empty winodw mesh node, and creat a CB-Tree
    // Shift the mask id in each module. ex: in the module cc_11, there are 3 patterns P1, P2, P3. and P1's mask_id is 0, P2 is 1, P3 is 2.
    // After action Op4, P1 will be 1, P2 will be 2, P3 will be 0.
    void Op4(); 
    void Op5(); // Rotate the module 180 degree. reverse the rotate flag.
    void Op6(); // Flip the module vertically. reverse the flip flag.

    /* Geometrical constraints checking */
    bool geometrical_constraint_checking();
    
    /* 
        Checking proximity constraint... 
        Proximity means the proximity modules should be a neighbour each other.
        In other words, boundaries of modules should be abuted.
    */
    bool proximity_checking();
    
    /*
         Checking boundary constraint... 
         Boundary means the module should be placed on the boundary of the window.
         In other words. the boundaries of window and module should be abuted.
    */
    bool boundary_checking();

    /*
        Checking fixed_boundary constraint... 
        Fixed_boundary is same as Boundary. Additional value is Rotation and flip.
        It should be placed in boundary with fixed rotation and flip.
    */
    bool fixed_boundary_checking();

    /*
        Checking symmetry constraint... 
        Symmetry means the module should be placed symmetrically about the axis.
    */
    bool symmetry_checking();

    /* FIN-FET constraints checking */

    /* Checking look_ahead_density constraint... 
        Before packing, we check the density with the modules information that are included in the WB-Tree.
        It avoids the infeasible solutions.
     */
    bool look_ahead_density_checking();

    /* 
        Checking global_mask_density constraint... 
        After packing, we check the density with the placement of modules.
        It avoids the mask density unbalanced solutions.
    */
    bool global_mask_density_checking();

    /* Checking local_mask_density constraint... 
        We check the density with the placement of modules in Local level.
        In other words, we create a local area called BIN based on the dense patterns and check the density balances between them.
    */
    bool local_mask_density_checking();
    
    /* 
        Check the pattern is isolated or densed for the local mask density checking. 
    */
    bool isIsolated(Layout_Pattern s);

    /* Calculate the mask density in every Bin level for the local mask density checking. */
    double get_mask_density_in_bin(Rect bin, Layout_Pattern s);

    /* Get mask area in window region */
    double get_mask_area(int w_id, int mask_id);
    
    /* After perturb operation, mask numbers of some layout patterns could be shifted. So we get the modified mask numbers. */
    void get_mask_id();


    /* Packing. Every CB-Tree's [modules_info]s are collected and positions are corrected and stored into WB-Tree's [modules_info] */
    void self_aligned_packing();

    /* Find module with module name */
    int  find_mod(char* module_name="randomly");

    /* Find Window mesh node with some parameters */
    WBNode* find_window_random();
    WBNode* find_wbnode(CB_Tree* cbtree);
    WBNode* find_wbnode(int mod_id);

    /* Calculate cost */
    double calculate_cost();
    
    /* Calculate the wirelength. scaleIOPad function makes the scale the IO pad for calculating wirelength */
    void    scaleIOPad();
    double  calcWireLength();

    /* Calculate the Out of Bound Area, that means the modules area which is out bounded of the window region. */
    double  calcOutBoundArea();

    /* Calculate the Violation cost. mask_conflict cost + local_mask_density cost + global_mask_density cost */
    double  calcViolationCost();

    /* Calculate the mask conflict cost */
    double  calcMaskConfictCost();
    double calculate_Ncft(int mod_id, int pattern_id); // calculate the number of patterns which occurs mask conflict.
    
    /*
        Output the result info to the .res file and console.
    */
    void    list_information();

    /*
        Calculate the white space of the whole region.
    */
    double  get_dead_space();

    /*
        Copy the WB-Tree. This function is used for Recovering Good solution.
    */
    void    copy_tree(vector<Node> *tree_o, vector<Node> *tree);

    /*
        Output the WB-Tree's information to the console.
    */
    void    show_tree();

    /* Get the preplaced modules which they are placed in the sepcific window region. */
    vector<PREPLACED>  get_preplaced_module(WBNode *wbnode);

    /* Generate the Matlab execution file. */
    void          output();

    double                  cost;           // Cost of the current solution.
    Modules                 modules;        // modules information. module's size, width, height.
    double                  Area;           // Area of the WB-Tree.
    double                  WireLength;     // WireLength of the current solution.
    double                  TotalArea;      // Total sum of modules' areas.
    double                  MDD;            // Mask_Density_Difference value.

    /* Solution saves the WB_Tree information. */
    struct Solution{
        vector<WBNode>      wbnodes;        // window mesh nodes.
        double              cost;           // cost of the current solution.          
        Solution() { cost = 1; }
        void clear() { cost = 1;
                       for (int i = 0; i < wbnodes.size(); i++)
                           if(wbnodes[i].cbtree){
                               vector<Node> t = *(vector<Node> *)wbnodes[i].cbtree;
                               t.clear();
                           }
                     }
    };

    /* Recover the solution. means the back to the saved WB-Tree status. */
    void recover(Solution &sol);

    /* Save the solution. means keep the current WB-Tree's information. */
    void keep_sol(Solution &sol);

    /* best_sol- the best solution, last_sol - the last_sol */
    Solution                best_sol, last_sol;

    /* packing_list saves the list of the CB-Trees. */
    vector<CB_Tree*>        packing_list;

private:

    /* input parameters. */
    float                   alpha;
    char                    filename[40];
    int                     times;
    int                     local;
    float                   term_temp;
    
    /* variables for parsing input informations. */
    ifstream                fs;
    char                    line[100],t1[40],t2[40];
    map<string,int>         net_table;                  // connection information with pins of the modules

    /* WB_Tree's modules' information, it stores the position of module, layout patterns, rotation, flip information. */
    Modules_Info            modules_info;

    /* layout patterns Map,  they are maped with the key as the module's id */
    map<int,Layout_Patterns> layout_patterns;

    /* module's number */
    int                     modules_N;

    /* 
        The Root Module doesn't mean the root of the tree. 
        In apte, there is a module that type is "parent".
        It includes the network information. 
        According to the YAL_Description, it is not a module, it's a connection information.
    */
    Module                  root_module;

    Nets                    network;                    // network information array.
    vector<vector<int>>     connection;                 // the connection information between each nets.
    int                     mask_num;                   // Masks' number.                       [from input file.]
    double                  mask_density_difference;    // Mask_Density_Difference.             [from input file.]
    int                     minimum_distance;           // Mask confliction Minimum_Distance    [from input file.]
    int                     win_h,win_w,bin_h,bin_w;    // Window/Bin Height, Width             [from input file.]
    int                     Dfin;                       // Distance between Fin lins.           [from input file.]
    Constraint             constraints;                 // Constraints information.

    vector<WBNode>          wbnodes;                    // Windows' mesh nodes.
    int                     width,height;               // Width and Height of the WB-Tree that generates.
};

#endif
