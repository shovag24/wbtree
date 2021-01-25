//---------------------------------------------------------------------------
#ifndef fplanH
#define fplanH
//---------------------------------------------------------------------------

//#define nullptr {(void*)0}

#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <cstdio>
#define NIL -1
//---------------------------------------------------------------------------
using namespace std;

/* It means the boundary of the layout pattern. */
struct Rect
{
  int left,right,bottom,top;
};

/* MINIMUM_SEPARATION */
struct MINIMUM_SEPERATION
{
    int mod;            // module id
    int dis;            // minimum_separation distance
};

/* Fixed Boundary constraint */
struct FIXED_BOUNDARY
{
    int mod;            // module id
    bool rotate;        // true : rotate 180 degree, false : not
    bool flip;          // flip : flip vertically, false : not
};

/* The Maximum separation constraint */
struct MAXIMUM_SEPERATION
{
    int mod1;           // first module id
    int mod2;           // second module id
    long dis;           // maximum distance between first and second
};

/* The Symmetry constraint */
struct SYMMETRY
{
    vector<int> sequance_pair1;     // the sequence array of the first modules' id
    vector<int> sequance_pair2;     // the sequence array of the second modules' id
    vector<int> self_symmetry_mods; // Self-symmetry modules's id
    int axis;                       // Symmetry Axis, ex: if the horizontal, axis means y cordinate. if the vertical, x cordinate
    bool is_vertical;               // true : vertical, false : horizontal
};

/* The preplaced constraint */
struct PREPLACED
{
    char mod[10] = "";              // preplaced module name. we get a preplaced module from the constraint file.
    int id;                         // preplaced module id.
    int x,y,rx,ry;                  // preplaced module's fixed position.
};

/* All constraints are stored in this structure. */
struct Constraint
{
    vector<MINIMUM_SEPERATION>  min_sep;
    vector<SYMMETRY>            symmetry;
    vector<int>                 proximity;
    vector<int>                 boundary;
    vector<FIXED_BOUNDARY>      fixed_boundary;
    vector<PREPLACED>           preplaced;
};

/* 
    Tile is used for Corner-stitiching Method in CB-Tree.
    Tiles are used in CB-Tree packing.
    It means the piece region that are divied by the module in the CB-Tree region.
 */
struct Tile
{
    int id;                         // tile id
    int cornerstich_x;              // corner-stitch x-cordinate.
    int cornerstich_y;              // corner-stitch y-cordinate.
    int rx,ry;                      // tile's right, top cordinate.
    bool solidTile;                 // true : module(solid tile), false : empty tile
    int bl,lb,tr,rt;                // relationship between other tiles, bl(bottom-left), lb(left-bottom), tr(top-right), rt(right-top)
    /* If the tile is removed or not created. */
    bool is_Nil()                                                   { return (cornerstich_x == NIL && cornerstich_y == NIL && rx == NIL && ry == NIL);}
    /* remove the tile */
    void set_Nil()                                                  { bl = lb = tr = rt = cornerstich_x = cornerstich_y = rx = ry = NIL;}
    /* set the tile */
    void set_Val(int c_x, int c_y, int r_x, int r_y, bool solid)    { cornerstich_x = c_x; cornerstich_y = c_y; rx = r_x; ry = r_y; solidTile = solid;}
    /* set relationship, bl,lb,tr,rt */
    void set_Con(int b_l, int l_b, int t_r, int r_t)                { bl = b_l, lb = l_b, tr = t_r, rt = r_t; }
};

typedef vector<Tile> Tiles;

/* Pin is the conception of the module. all pins are connected to other modules' pins. */
struct Pin{
    int mod;    // module id
    int net;    // net id
    int x,y;    // relative position
    int ax,ay;  // absolute position
    Pin(int x_=-1,int y_=-1){ x=x_,y=y_; }
};

// ---------------------------------------------------------------------

typedef vector<Pin> Pins;
typedef Pin* Pin_p;
typedef vector<Pin_p> Net;
typedef vector<Net > Nets;
enum Module_Type { MT_Hard, MT_Soft, MT_Reclinear, MT_Buffer };

// ---------------------------------------------------------------------

/* layout patterns in the module. Fin-fet technology */
struct Layout_Pattern
{
    Rect    layout;     // pattern's position and size.
    bool    internal;   // true : internal pattern, false : external pattern.
    int     mask_id;    // mask id
};

typedef vector<Layout_Pattern> Layout_Patterns;

// ----------------------------------------------------------------------

/* module data from the input file */
struct Module{
    int id;                 // module id
    char name[20];          // module name
    int width,height;       
    int x,y;              
    int area;
    Pins pins;
    // distance between module's top and first fin line (Dtop), module's bottom and last fin line (Dbot), between fins (Dfin)
    int Dtop,Dbot,Dfin;     
    Module_Type type;
};
typedef vector<Module> Modules;

// --------------------------------------------------------------------------

/* Module's information after placed. */
struct Module_Info{
    bool rotate, flip;      // rotate, and flip information true: action, false : not
    int x,y;                // x,y means the left-bottom cordinate of the module,
    int rx,ry;              // rx,ry means the right-top cordinate of the module,
    int Dbot,Dfin,Dtop;     // Dbot: distance between bottom line of module and the last fin line. Dtop : between top line of module and the first fin line, Dfin : fin distance.
    Layout_Patterns patterns; // Layout_patterns, its information includes the patterns' position.
};

// Impotant point is that the member variable "layout_patterns" and modules_info's "patterns" are different. 
// "layout_patterns" have a patterns' relative position based on the module. In other words, the pattern's location was specified assuming the module is on the edge of the point (0,0).
// But "patterns" have a corrected positions based on the real module's position from the packing scheme.
// For example, Let's there is a module [0,0,100,400] from the input file and there is a layout pattern [10,10,40,40]. (module is stored into "modules" and the pattern is stored into "layout_patterns")
// After packing, in the module's_info, that module's position is determined as [0+300, 0+500, 100+300, 400+500].
// Then the "pattern" variable in the module's_info will be [10+300, 10+500, 40+300, 40+500].
typedef vector<Module_Info> Modules_Info;

/* CB-Tree's node. */
struct Node{
    int id;
    Node *parent,*left,*right; // parent node, left/right child node.
    bool rotate,flip;   // rotate, flip module.
    // true : leaf node, false : not
    bool isleaf(){ return (left==nullptr && right==nullptr); } 
};

// CB-Tree's Parent class. Based on B*-Tree. FPlan class has a main member vairables and basic set functions.
// It is a main body of the CB-Tree.
class FPlan{
public:
    FPlan(float calpha);
    virtual void init()		=0;
    virtual void packing();
    virtual void perturb()	=0;
    virtual double getCost();

    int                     size()                      { return nodes_N;               }   // return the nodes' number
    int                     muduleSize()                { return modules_N;             }   // return the modules' number
    vector<Module_Info>     getModuleInfo()             { return modules_info;          }   // return modules information (position. etc...).
    double                  getTotalArea()              { return TotalArea;             }   // return sum of the modules' area
    double                  getArea()                   { return Area;                  }   // return CB-Tree's area
    int                     getWireLength()             { return WireLength;            }   // return CB-Tree's wirelength.
    double                  getWidth()                  { return Width;                 }   // return CB-Tree's width
    double                  getHeight()                 { return Height;                }   // return CB-Tree's height
    float                   getDeadSpace();

    vector<Module>          getModule()                 { return modules;               }   // return modules.
    Module                  getRootModule()             { return root_module;           }   // return the root_module(parent module). not a root of CB-Tree

    // set modules.
    void                    setModules(Modules modules) { this->modules = modules; modules_N = modules.size(); modules_info.resize(modules_N);}
    // set the Root Module (Parent module)
    void                    setRootModule(Module module){ this->root_module = module;   }
    // set the network.
    void                    setNetwork(int size)        { this->netsize = size;      }
    // set the layout patterns of modules.
    void                    setLayoutPatterns(map<int,Layout_Patterns> patterns);


    // information
    void                    list_information();         // output the CB-Tree's result in the console, but not used in WB-Tree
    void                    show_modules();             // shows the CB-Tree's modules in the console
    void                    normalize_cost(int);        // randomly placed the modules in CB-Tree. it is proceed once time when the CB-Tree is created.

    void                    create_network(int size);   // Create the network in CB-Tree.

    Node*                   nodes_root;                 // root node of the CB-Tree, not a Root Module.
    map<int, Layout_Patterns> layout_patterns;          // layout patterns variable.
    int                     modules_N;                  // total modules' number
    Modules                 modules;                    // modules data. it stores the module's basic information like size, width, height.
    Module                  root_module;                // Root Module. Doesn't mean the root node of the Tree. According to the YAL_Description, we call a module that type is "parent" as a ROOT Module.
    Modules_Info            modules_info;               // modules information position, flip, rotate, layout patterns' position, fin line. etc..(in the CB-Tree)
    // "modules_info" variable is for keeping the result of the packing scheme. it stores the determined modules and patterns positions.

    vector<MINIMUM_SEPERATION> min_seps;                // Minimum_Separation constraints.
    vector<FIXED_BOUNDARY>  fb;                         // Fixed Boundary constraints.
    vector<PREPLACED>       preplaced;                  // Preplaced consraints
    
// --------------------------------------------------------------------------------

protected:
    void                    clear();                    // initialize the CB-Tree
    double                  calcWireLength();           // Calculate the wirelength in the CB-Tree. [not proceed in WB-Tree]
    void                    scaleIOPad();               // Calculate the wirelength in the CB-Tree. [not proceed in WB-Tree]

    double                  Area;                       // cb-Tree's area
    double                  Width,Height;               // CB-Tree's width and height
    int                     WireLength;                 // CB-Tree's wirelength.                        [no used in WB-Tree]
    double                  TotalArea;                  // sum of modules's area in CB-Tree.            [not used in WB-Tree.]
    int                     nodes_N;                    // nodes's number
    Nets                    network;                    // network in CB-Tree.                          [not used in WB-Tree]
    double                  norm_area, norm_wire;       // average area, average wirelength in the CB-Tree [not used in WB-Tree]
    float                   cost_alpha;                 // [not used in WB-Tree]
    vector<vector<int>>     connection;                 // connection information between the modules.  [not used in WB-Tree]

//-------------------------------------------------------------------------------------------------

private:
    int             netsize;
    string          filename;
};


void error(char *msg,char *msg2="");
bool rand_bool();
float rand_01();
double seconds();

//---------------------------------------------------------------------------
#endif
