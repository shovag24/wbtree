#include "wbtree.h"

// Safe delete. manage memory effectness
#define SAFE_DELETE(p) { \
    if(p==NULL) { \
    std::string str="\ndelete on nullptr: "; \
    str+=__FILE__; str+="//"; str+=std::to_string(__LINE__);str+="\n"; \
    printf("%s",str.c_str()); \
    throw std::runtime_error("DELETE ON NULLPTR"); \
    } \
    else{ \
    delete p; \
    p=NULL; \
    } \
    }

using namespace std;
//-------------------------------------------------------------------------
/* Init WB-Tree, Preprocessing. */
//-------------------------------------------------------------------------
// init the WB-Tree
void WB_Tree::init(float alpha, char *filename, int times, int local, float term_temp)
{
    //[1]. Save the input parameters.
    WB_Tree::alpha = alpha;
    strcpy(WB_Tree::filename, filename);
    WB_Tree::times = times;
    WB_Tree::local = local;
    WB_Tree::term_temp = term_temp;

    //[2]. Read the module information of input file.
    read_module_info();

    //[3]. Read the design information and design constraint of input file.
    read_design_rules();

    //[4]. Read the geometrical constraint information of input file.
    read_constraint();

    //[5]. Module mask assignment.
    mask_assignment();

    //[6]. Preprocessing.
    preprocessing();

    normalize_cost(10);
}

/* Greedy mask assignment.  */
void WB_Tree::mask_assignment()
{
    int id;
    for(int i = 0; i < modules_N; i++)
    {
        id = 0;
        // the module has no pattern.
        if(layout_patterns[i].empty())
            continue;
        // assign initial number of mask.
        for(int j = 0; j < layout_patterns[i].size(); j++)
        {
            layout_patterns[i][j].mask_id = id % mask_num;
            id++;
        }
    }
}

void WB_Tree::preprocessing()
{
    //[1]. Extract critical layout patterns.
    extract_critical_layout_patterns();

    //[2]. Construct Initial WB-Tree
    construct_wb_tree();
}

/* determine the internal pattern and external pattern. External patterns are used in mask conflict constraint handling */
void WB_Tree::extract_critical_layout_patterns()
{
    for(int i = 0; i < modules_N; i++)
    {
        if(layout_patterns[i].empty())
            continue;
        for(int j = 0; j < layout_patterns[i].size(); j++)
        {
            Rect t = layout_patterns[i][j].layout;
            // If the layout pattern is in the min_mask_distance boundary of module.
            if(t.top > (modules[i].height - minimum_distance) ||
                    t.left < minimum_distance || t.bottom < minimum_distance ||
                    t.right > (modules[i].width - minimum_distance))
            {
                layout_patterns[i][j].internal = false;
            }
            // internal layout pattern
            else
            {
                layout_patterns[i][j].internal = true;
            }
        }
    }
}

void WB_Tree::construct_wb_tree()
{
    int rows, cols;
    //[1]. Create CB-Tree and initialize it.
    CB_Tree *cbt = new CB_Tree(alpha);
    init_cbtree(*cbt);
    cbt->init();
    double total_area = 0;
    //[2]. Create Windows Mesh Tree.
        //[2.1] calculate the total area.
    for(int i = 0; i < modules_N; i++)
    {
        total_area += modules[i].area;
    }
    for(int i = 0; i < constraints.preplaced.size(); i++)
    {
        total_area += (constraints.preplaced[i].rx - constraints.preplaced[i].x) * (constraints.preplaced[i].ry - constraints.preplaced[i].y);
    }
        //[2.2] calculate the infeasible window number.(total area's 125% area is used for enough floorplanning.)
    int t = ceil((total_area * 1.25) / (win_w * win_h));
        //[2.3] get the row num and col num.
    rows = cols = sqrt(t);
        //[2.4] create window.
    wbnodes.reserve(vector_max_size);
    for(int i = 0; i < rows * cols; i++)
    {
        WBNode wbnode;
        wbnode.col = i % cols;
        wbnode.row = i / cols;
        wbnode.cbtree = nullptr;
        wbnodes.push_back(wbnode);
    }

    //[3]. Construct WB-Tree.
    int rand_index = rand() % wbnodes.size();
    wbnodes[rand_index].cbtree = cbt;   
    wbnodes[rand_index].cbtree->preplaced = get_preplaced_module(&wbnodes[rand_index]);
    packing_list.push_back(cbt);
}

/* Get the preplaced modules which is belong to the "*wbnode" in preplace constraints */
vector<PREPLACED> WB_Tree::get_preplaced_module(WBNode *wbnode)
{
    vector<PREPLACED> preplaceds;
    int x,y,rx,ry, base_x, base_y;
    base_x = wbnode->col * win_w;
    base_y = wbnode->row * win_h;
    for(int i = 0; i < constraints.preplaced.size(); i++)
    {

        x = constraints.preplaced[i].x;
        y = constraints.preplaced[i].y;
        rx = constraints.preplaced[i].rx;
        ry = constraints.preplaced[i].ry;
        // if the preplaced module is in this window region.
        if( (x >= base_x && rx <= ( base_x + win_w )) && (y >= base_y && ry <= ( base_y + win_h )) )
        {
            PREPLACED p;
            strcpy(p.mod, constraints.preplaced[i].mod);
            p.x = x - base_x;
            p.y = y - base_y;
            p.rx = rx - base_x;
            p.ry = ry - base_y;
            preplaceds.push_back(p); // add this ppm module.
        }

    }

    return preplaceds;
}

/* Set the module, create network, layout patterns and constraints in CBTree which is created. */
void WB_Tree::init_cbtree(CB_Tree &fp)
{
    //[1]. set base information to the CB-Tree
    fp.setModules(modules);
    fp.setRootModule(root_module);
    fp.create_network(net_table.size());
    fp.setLayoutPatterns(layout_patterns);
    fp.min_seps = constraints.min_sep;
    fp.fb = constraints.fixed_boundary;
}

//-------------------------------------------------------------------------
/* Perturbation */
//-------------------------------------------------------------------------

/* Perturb without constraint handling. */
void WB_Tree::perturb()
{
    if(rand_bool())
        Op1();
    if(rand_bool())
        Op2();
    if(rand_bool())
        Op3();
    if(rand_bool())
        Op4();
    if(rand_bool())
        Op5();
    if(rand_bool())
        Op6();

    self_aligned_packing();
}

/* Perturb with constraint handling */
void WB_Tree::perturbation()
{
//[1]. Perturb
    if(rand_bool())
        Op1();
    if(rand_bool())
        Op2();
    if(rand_bool())
        Op3();
    if(rand_bool())
        Op4();
    if(rand_bool())
        Op5();
    if(rand_bool())
        Op6();

    if(!look_ahead_density_checking()) 
    {
        // fails look ahead density checking.
        perturbation();
    }

    if(!geometrical_constraint_checking())
    {
        // fails geometrical constraints
        perturbation();
    }
//[2].Packing    
    // packing stage.
    self_aligned_packing();
    
    //global density checking
    if(!global_mask_density_checking())
    {
        // fails the global mask density checking
        perturbation();
    }

    //local density checking.
    if(!local_mask_density_checking())
    {
        //fails the local mask density checking
        perturbation();
    }
}



//-------------------------------------------------------------------------
/*===Geometrical constraint checking=====================================*/
//-------------------------------------------------------------------------

bool WB_Tree::geometrical_constraint_checking()
{
    //Geometrical constraint.
    if(!proximity_checking())
        return false;
    if(!boundary_checking())
        return false;
    if(!fixed_boundary_checking())
        return false;
    if(!symmetry_checking())
        return false;
    return true;
}

bool WB_Tree::symmetry_checking()
{

    double x1,y1,x2,y2,left,right,top,bottom;
    int mid;
    //[1]. Recurit the symmetry constraints list.
    for(int i = 0; i < constraints.symmetry.size(); i++)
    {
        for(int j = 0; j < constraints.symmetry[i].sequance_pair1.size(); j++)
        {
            //[2]. Determine the first module's center point in sequance_pair.
            mid = constraints.symmetry[i].sequance_pair1[j];
            left = modules_info[mid].x > modules_info[mid].rx ? modules_info[mid].rx : modules_info[mid].x;
            bottom = modules_info[mid].y > modules_info[mid].ry ? modules_info[mid].ry : modules_info[mid].y;
            right = modules_info[mid].x > modules_info[mid].rx ? modules_info[mid].x : modules_info[mid].rx;        
            top = modules_info[mid].y > modules_info[mid].ry ? modules_info[mid].y : modules_info[mid].ry;
            
            x1 = right - ( right - left ) / 2;
            y1 = top - ( top - bottom ) / 2;

            //[3]. Determine the second module's center point in sequance_pair.
            mid = constraints.symmetry[i].sequance_pair2[j];
            left = modules_info[mid].x > modules_info[mid].rx ? modules_info[mid].rx : modules_info[mid].x;
            bottom = modules_info[mid].y > modules_info[mid].ry ? modules_info[mid].ry : modules_info[mid].y;
            right = modules_info[mid].x > modules_info[mid].rx ? modules_info[mid].x : modules_info[mid].rx;        
            top = modules_info[mid].y > modules_info[mid].ry ? modules_info[mid].y : modules_info[mid].ry;
            
            x2 = right - ( right - left ) / 2;
            y2 = top - ( top - bottom ) / 2;

            //[4]. check vertical symmetry in symmetry pair.
            if(constraints.symmetry[i].is_vertical)
            {
                if( ( x1 + x2 ) != (2 * constraints.symmetry[i].axis) || y1 != y2 )
                    return false;
            }
            //[5]. check horizontal symmetry in symmetry pair
            else
            {
                if( ( y1 + y2 ) != (2 * constraints.symmetry[i].axis) || x1 != x2 )
                    return false;
            }
        }
        //[6]. Self symmetry checking.
        for(int j = 0; j < constraints.symmetry[i].self_symmetry_mods.size(); j++)
        {
            //[7]. Determine the center point of self symmetry module.
            mid = constraints.symmetry[i].self_symmetry_mods[j];
            left = modules_info[mid].x > modules_info[mid].rx ? modules_info[mid].rx : modules_info[mid].x;
            bottom = modules_info[mid].y > modules_info[mid].ry ? modules_info[mid].ry : modules_info[mid].y;
            right = modules_info[mid].x > modules_info[mid].rx ? modules_info[mid].x : modules_info[mid].rx;        
            top = modules_info[mid].y > modules_info[mid].ry ? modules_info[mid].y : modules_info[mid].ry;
            x1 = right - ( right - left ) / 2;
            y1 = top - ( top - bottom ) / 2;
            //[8]. check vertical symmetry in self symmetry.
            if(constraints.symmetry[i].is_vertical)
            {
                if( x1 != constraints.symmetry[i].axis )
                    return false;
            }
            //[8]. check horizontal symmetry in self symmetry.
            else
            {
                if( y1 != constraints.symmetry[i].axis )
                    return false;
            }
        }

    }
    return true;
}

bool WB_Tree::proximity_checking()
{
    int distance = 0;
    int id,xdis,ydis,left,right,top,bottom,left_t,right_t,top_t,bottom_t;
    int mod1,mod2;
   
    for(int i = 0; i < constraints.proximity.size(); i++)
    {
        int t = 0;
        //[1]. determine the module i's boundary.
        mod1 = constraints.proximity[i];
        left = modules_info[mod1].x;
        right = modules_info[mod1].rx;
        top = modules_info[mod1].ry;
        bottom = modules_info[mod1].y;
        for(int j = 0; j < constraints.proximity.size(); j++)
        {
            if(i == j)
                continue;
            //[2] determine the module j's boundary.
            mod2 = constraints.proximity[j];
            left_t = modules_info[mod2].x;
            right_t = modules_info[mod2].rx;
            bottom_t = modules_info[mod2].y;
            top_t = modules_info[mod2].ry;

            //[3]. determine the minimum distance between i and j modules.
            xdis = min(min(abs(left_t - left) , abs(left_t - right)), min(abs(right_t - left) , abs(right_t - right))); 
            ydis = min(min(abs(top_t - top) , abs(top_t - bottom)), min(abs(bottom_t - top) , abs(bottom_t - bottom)));
            distance = sqrt(pow(xdis,2) + pow(ydis,2));

            //[4]. check the two moudles are abuted.
            if(distance == 0){
                // two modules are abuted.
                t++;
            }
        }
        if(t == 0)
        {
            // Module i is distanced from their proximity modules.
            return false;
        }
    }
    
    return true;
}

bool WB_Tree::boundary_checking()
{
    int id;

    for(int i = 0;  i < constraints.boundary.size(); i++)
    {
        id = constraints.boundary[i];
        WBNode *wbnode = find_wbnode(id);
        //[1] check the module's boundary is the boundary of window.
        if(modules_info[id].x == wbnode->col * win_w 
        || modules_info[id].y == wbnode->row * win_h 
        || modules_info[id].rx == (wbnode->col + 1) * win_w 
        || modules_info[id].ry == (wbnode->row + 1) * win_h) 
        {
            //cout<<"The i'th boundary constraint is Okay"<<endl;
        }else
        {
            return false;
        }
    }
    return true;
}

bool WB_Tree::fixed_boundary_checking()
{
    int id;

    for(int i = 0;  i < constraints.boundary.size(); i++)
    {
        id = constraints.fixed_boundary[i].mod;
        WBNode *wbnode = find_wbnode(id);
        //[1] check the module's boundary is the boundary of window.
        if(modules_info[id].x != wbnode->col * win_w 
        && modules_info[id].y != wbnode->row * win_h
        && modules_info[id].rx != (wbnode->col + 1) * win_w  
        && modules_info[id].ry != (wbnode->row + 1) * win_h)
        {
            return false;
        }
        if(modules_info[id].rotate != constraints.fixed_boundary[id].rotate || modules_info[id].flip != constraints.fixed_boundary[id].flip)
        {
            return false;
        }
    }

    return true;
}

//-------------------------------------------------------------------------
/*===Fin-Fet desing constraint checking=====================================*/
//-------------------------------------------------------------------------

bool WB_Tree::look_ahead_density_checking()
{
    double density_difference,t1,t2;
    double area = win_w * win_h;
    get_mask_id();
    for(int i = 0; i < packing_list.size(); i++)
    {
        for(int j = 0; j < mask_num; j++)
        {
            // t1,t2 = total area of layout patterns which mask id is 'j'. 'i' is the window id.
            t1 = get_mask_area(0,j) / area;
            t2 = get_mask_area(i,j) / area;
            // calculate the density difference.
            if(t1 == 0 || t2 == 0)
                density_difference = 0;
            else
                density_difference = t1 >= t2 ? (t1-t2) : (t2-t1);
            if(density_difference > mask_density_difference)
                return false;
        }       
    }
    return true;
}

bool WB_Tree::local_mask_density_checking()
{
    int x,y;
    double d = INT_MIN,t1 = 0,t2,tmp;
    double area = win_w * win_h;
    Layout_Patterns S;
    // make the literal of layout patterns orderby horizontal and vertical. wbnodes vector is ordered by rows and cols.
    // col means vertical,row means horizontal.
    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
            continue;
        vector<Node*> nodes = wbnodes[i].cbtree->allnodes();
        for(int j = 0; j < nodes.size(); j++)
        {
            for(int k = 0; k < modules_info[nodes[j]->id].patterns.size(); k++)
            {
                S.push_back(modules_info[nodes[j]->id].patterns[k]);    // make the stack of the layout patterns.
            }
        }

    }
    // check the pattern is isolated, and otherwise, calculate the density difference in bin.
    for(int i = 0; i < S.size(); i++)
    {
        if(!isIsolated(S[i]))
        {
            //[1]. create a BIN
            Rect bin;
            x = S[i].layout.right - (S[i].layout.right - S[i].layout.left) / 2; 
            y = S[i].layout.top - (S[i].layout.top - S[i].layout.bottom) / 2; 
            bin.left = x - bin_w / 2;
            bin.right = x + bin_w / 2;
            bin.bottom = y - bin_h / 2;
            bin.top = y + bin_h / 2;
            //[2]. mask density difference.
            t2 = get_mask_density_in_bin(bin, S[i]) / area;
            
            if(t1 == 0)
                t1 = t2;
            
            if(t1 == 0 || t2 == 0)
                tmp = 0;
            else
                tmp = t1 >= t2 ? (t1-t2)/t2 : (t2-t1)/t1;
            if(d < tmp);
                d = tmp;
        }
    }

    if(d > mask_density_difference)
    {
        return true;
    }

    return true;
}

bool WB_Tree::global_mask_density_checking()
{
   double density_difference,t1,t2;
    double area = win_w * win_h;
    for(int i = 0; i < packing_list.size(); i++)
    {
        for(int j = 0; j < mask_num; j++)
        {
            // t1,t2 = total area of layout patterns which mask id is 'j'. 'i' is the window id.
            t1 = get_mask_area(0,j) / area;
            t2 = get_mask_area(i,j) / area;
            // calculate the density difference.
            if(t1 == 0 || t2 == 0)
                density_difference = 0;
            else
                density_difference = t1 >= t2 ? (t1-t2) : (t2-t1);
            if(density_difference > mask_density_difference) // calculated density_dif > maximum_density_dif
                return false;
        }       
    }
    return true;
}

/* get mask density value in bin lebel */
double WB_Tree::get_mask_density_in_bin(Rect bin, Layout_Pattern s)
{
    double area = 0;
    for(int i = 0; i < modules_N; i++)
    {
        for(int j = 0; j < modules_info[i].patterns.size(); j++)
        {
            Layout_Pattern p = modules_info[i].patterns[j];
            if(p.mask_id != s.mask_id)
                continue;
            if(p.layout.left > bin.right || p.layout.right < bin.left || p.layout.top < bin.bottom || p.layout.bottom > bin.top)
            {
                // pattern is not in the bin region.
                continue;
            }
            else
            {
                // calculate the pattern's area and add it
                area += (p.layout.right - p.layout.left) * (p.layout.top - p.layout.bottom); 
            }
        }
    }
    return area;
}

/* Check the pattern is isolated or not. */
bool WB_Tree::isIsolated(Layout_Pattern s)
{
    int x,y;
    //[1] make a bin.
    Rect bin;
    x = s.layout.right - (s.layout.right - s.layout.left) / 2; 
    y = s.layout.top - (s.layout.top - s.layout.bottom) / 2; 
    bin.left = x - bin_w / 2;
    bin.right = x + bin_w / 2;
    bin.bottom = y - bin_h / 2;
    bin.top = y + bin_h / 2;

    //[2]. check the pattern is isolated or not.
    for(int i = 0; i < modules_N; i++)
    {
        for(int j = 0; j < modules_info[i].patterns.size(); j++)
        {
            Layout_Pattern p = modules_info[i].patterns[j];
            if(p.mask_id != s.mask_id || p.layout.left == s.layout.left)
                continue;
            if(p.layout.left > bin.right || p.layout.right < bin.left || p.layout.top < bin.bottom || p.layout.bottom > bin.top)
            {
                // the pattern is not in the Bin region
                continue;
            }
            else
            {
                //there is another pattern in the bin. so this pattern is not isolated.
               return true;
            }
        }
    }
    // the pattern is isolated. there are no other patterns in the bin region that made by this pattern
    return false;
}

// calculate sum of the mask patterns area in windows level.
double WB_Tree::get_mask_area(int w_id, int mask_id) 
{
    vector<Node*> nodes = packing_list[w_id]->allnodes();
    double area = 0;
    for(int i = 0; i < nodes.size(); i++)
    {
        //cout<<"pattern_size : "<<endl;
        for(int j = 0; j < modules_info[nodes[i]->id].patterns.size(); j++)
        {
            if(modules_info[nodes[i]->id].patterns[j].mask_id == mask_id)
                area += (modules_info[nodes[i]->id].patterns[j].layout.right
                     - modules_info[nodes[i]->id].patterns[j].layout.left) 
                     * (modules_info[nodes[i]->id].patterns[j].layout.top
                     - modules_info[nodes[i]->id].patterns[j].layout.bottom);
        }
    }
    return area;
}

//-------------------------------------------------------------------------
/* Main perturb actions */
//-------------------------------------------------------------------------
// Move node from one CB-Tree to another
void WB_Tree::Op1()
{
    int i,j,mid;
    //[0]. If there is only one CB-Tree, it is impossible to do Op1().
    if(packing_list.size() == 1)
        return;
    //[1]. Select random CB-Tree.
    do{
        i = rand() % packing_list.size();
        j = rand() % packing_list.size();
    }while(i == j);

    //[2]. Move node from i-th cbtree to j-th cbtree
    if(packing_list[i]->allnodes().size() == 1)
    {
        WBNode *wbnode = find_wbnode(packing_list[i]);
        mid = packing_list[i]->take_node_random();
        packing_list[j]->insert_node_by_id(packing_list[j]->find_node_random(), mid);

        wbnode->cbtree = nullptr;
        //delete b_trees[i];
        delete packing_list[i];
        packing_list.erase(packing_list.begin() + i);
    }else
    {
        mid = packing_list[i]->take_node_random();
        packing_list[j]->insert_node_by_id(packing_list[j]->find_node_random(), mid);
    }
}

//Swap the nodes between two CB-Trees.
void WB_Tree::Op2()
{
    int i, j, mid_i, mid_j;
    //[0]. If there is only one CB-Tree, it is impossible to do Op2().
    if(packing_list.size() == 1)
        return;

    // [1]. generate 2 different indices in [0 until b_tress.size()]
    do
    {
        i = rand() % packing_list.size();
        j = rand() % packing_list.size();
    } while (i == j);

    mid_i = packing_list[i]->find_node_random()->id;
    mid_j = packing_list[j]->find_node_random()->id;

    // [2]. swap_node
    packing_list[i]->insert_node_by_id(packing_list[i]->find_node_random(), mid_j);
    packing_list[i]->take_node(mid_i);
    packing_list[j]->insert_node_by_id(packing_list[j]->find_node_random(), mid_i);
    packing_list[j]->take_node(mid_j);
}

// Move a node to a empty window and Create a CB-Tree.
void WB_Tree::Op3()
{
    int i, mid;
    // [1].find window node which has no cb-tree
    WBNode *wbnode = find_window_random();
    if (wbnode == nullptr)
        return;

    // [2]. delete from the first CB-Tree

    i = rand() % packing_list.size();
    if (packing_list[i]->allnodes().size() == 1)
    {
        WBNode *del_wbnode = find_wbnode(packing_list[i]);
        del_wbnode->cbtree = nullptr;
        mid = packing_list[i]->take_node_random();
        delete packing_list[i];
        packing_list.erase(packing_list.begin() + i);
    }
    else
    {
        mid = packing_list[i]->take_node_random();
    }
    // [3]. make new CB-Tree
    CB_Tree *cbt = new CB_Tree(alpha);
    init_cbtree(*cbt);
    vector<int> inds;
    inds.push_back(mid);
    cbt->preplaced = get_preplaced_module(wbnode);
    cbt->init_with_node_indices(inds);
    // [4]. register root node
    wbnode->cbtree = cbt;
    packing_list.push_back(wbnode->cbtree);
}

// Shift the mask id of a module.
void WB_Tree::Op4()
{
    int i = rand() % packing_list.size();
    int mid;
    CB_Tree *cbtree = packing_list[i];
    Node *node = cbtree->find_node_random();
    mid = node->id;
    //[1]. Shift the mask id.
    for(i = 0; i < cbtree->layout_patterns[mid].size(); i++)
    {
        int mask_new_id = (cbtree->layout_patterns[mid][i].mask_id + 1) % mask_num;
        cbtree->layout_patterns[mid][i].mask_id = mask_new_id;
        layout_patterns[mid][i].mask_id = mask_new_id;
    }
}

// Flip the module
void WB_Tree::Op5()
{
    int i = rand() % packing_list.size();
    CB_Tree *cbtree = packing_list[i];
    Node *node;

    node = cbtree->find_node_random();
    //[1]. Flip the module.
    node->flip = !node->flip;
}

// Rotate the module.
void WB_Tree::Op6()
{
    int i = rand() % packing_list.size();
    CB_Tree *cbtree = packing_list[i];
    Node *node;
    node = cbtree->find_node_random();
    //[1]. Rotate the module.
    node->rotate = !node->rotate;
}

//-------------------------------------------------------------------------
/* Self-aligned-Packing */
//-------------------------------------------------------------------------

// Mask information for all modules in all the CB-Trees is aggregated and kept in the modules_info variable.
void WB_Tree::get_mask_id()
{
    int base_x,base_y;
    for(int i = 0; i < packing_list.size(); i++)
    {
        CB_Tree *cbtree = packing_list[i];
        WBNode *wbnode = find_wbnode(cbtree);
        cbtree->packing();
    }

    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
            continue;

        //[1]. calculate window's postition.
        base_x = wbnodes[i].col * win_w;
        base_y = wbnodes[i].row * win_h;
        //[2]. Correct the postion.
        vector<Node*> nodes = wbnodes[i].cbtree->allnodes();
        for(int j = 0; j < nodes.size(); j++)
        {
            modules_info[nodes[j]->id].x = wbnodes[i].cbtree->modules_info[nodes[j]->id].x + base_x;
            modules_info[nodes[j]->id].y = wbnodes[i].cbtree->modules_info[nodes[j]->id].y + base_y;
            modules_info[nodes[j]->id].rx = wbnodes[i].cbtree->modules_info[nodes[j]->id].rx + base_x;
            modules_info[nodes[j]->id].ry = wbnodes[i].cbtree->modules_info[nodes[j]->id].ry + base_y;

            //[3]. Correct the layout pattern's position.
            if(modules[nodes[j]->id].Dfin != 0){
                for(int k = 0; k < modules_info[nodes[j]->id].patterns.size(); k++)
                {
                    modules_info[nodes[j]->id].patterns[k].layout.left = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.left + base_x;
                    modules_info[nodes[j]->id].patterns[k].layout.right = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.right + base_x;
                    modules_info[nodes[j]->id].patterns[k].layout.bottom = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.bottom + base_y;
                    modules_info[nodes[j]->id].patterns[k].layout.top = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.top + base_y;

                    modules_info[nodes[j]->id].patterns[k].mask_id = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].mask_id;
                    modules_info[nodes[j]->id].patterns[k].internal = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].internal;

                    modules_info[nodes[j]->id].Dbot = wbnodes[i].cbtree->modules_info[nodes[j]->id].Dbot;
                    modules_info[nodes[j]->id].Dfin = wbnodes[i].cbtree->modules_info[nodes[j]->id].Dfin;
                    modules_info[nodes[j]->id].Dtop = wbnodes[i].cbtree->modules_info[nodes[j]->id].Dtop;

                }
            }
        }
    }
}

// Modules information in all the CB-Trees is aggregated and kept in the modules_info variable.
void WB_Tree::self_aligned_packing()
{
    int base_x,base_y,w,h;
    int min_x = INT_MAX ,max_rx = INT_MIN, min_y = INT_MAX ,max_ry = INT_MIN;
    for(int i = 0; i < packing_list.size(); i++)
    {
        CB_Tree *cbtree = packing_list[i];
        WBNode *wbnode = find_wbnode(cbtree);
        cbtree->packing();
    }

    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
            continue;

        //[1]. calculate window's postition.
        base_x = wbnodes[i].col * win_w;
        base_y = wbnodes[i].row * win_h;
        //base_y = ceil((float)base_y / 300) * 300;
        //[2]. Correct the postion.
        vector<Node*> nodes = wbnodes[i].cbtree->allnodes();
        for(int j = 0; j < nodes.size(); j++)
        {
            modules_info[nodes[j]->id].x = wbnodes[i].cbtree->modules_info[nodes[j]->id].x + base_x;
            modules_info[nodes[j]->id].y = wbnodes[i].cbtree->modules_info[nodes[j]->id].y + base_y;
            modules_info[nodes[j]->id].rx = wbnodes[i].cbtree->modules_info[nodes[j]->id].rx + base_x;
            modules_info[nodes[j]->id].ry = wbnodes[i].cbtree->modules_info[nodes[j]->id].ry + base_y;

            min_x = min(min_x, modules_info[nodes[j]->id].x);
            max_rx = max(max_rx, modules_info[nodes[j]->id].rx);
            min_y = min(min_y, modules_info[nodes[j]->id].y);
            max_ry = max(max_ry, modules_info[nodes[j]->id].ry);
            //[3]. Correct the layout pattern's position.
            if(modules[nodes[j]->id].Dfin != 0){
                for(int k = 0; k < modules_info[nodes[j]->id].patterns.size(); k++)
                {
                    modules_info[nodes[j]->id].patterns[k].layout.left = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.left + base_x;
                    modules_info[nodes[j]->id].patterns[k].layout.right = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.right + base_x;
                    modules_info[nodes[j]->id].patterns[k].layout.bottom = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.bottom + base_y;
                    modules_info[nodes[j]->id].patterns[k].layout.top = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].layout.top + base_y;

                    modules_info[nodes[j]->id].patterns[k].mask_id = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].mask_id;
                    modules_info[nodes[j]->id].patterns[k].internal = wbnodes[i].cbtree->modules_info[nodes[j]->id].patterns[k].internal;

                    modules_info[nodes[j]->id].Dbot = wbnodes[i].cbtree->modules_info[nodes[j]->id].Dbot;
                    modules_info[nodes[j]->id].Dfin = wbnodes[i].cbtree->modules_info[nodes[j]->id].Dfin;
                    modules_info[nodes[j]->id].Dtop = wbnodes[i].cbtree->modules_info[nodes[j]->id].Dtop;

                }
            }
        }
    }

    width = max_rx - min_x;
    height = max_ry - min_y;
    Area = width * height;

    calcWireLength();
}

//-------------------------------------------------------------------------
/* Calculate the Cost */
//-------------------------------------------------------------------------

double WB_Tree::calculate_cost()
{
    double A,W,O,V,alpha = 0.25,gamma = 0.25,beta = 0.25,rammda = 0.25;

    A = Area / TotalArea;       // Area cost.

    W = WireLength*1e-6;        // Wirelength

    O = calcOutBoundArea();     // Out of bound area cost

    V = calcViolationCost();    // violation cost
  
    cost = alpha * A + beta * W + gamma * O + rammda * V;

    return cost;
}

double WB_Tree::calcViolationCost()
{
    int x,y;
    double Mcft,Gmd = 0,Lmd = 0;
    Mcft = calcMaskConfictCost();
    double area = 0,d,t1,t2,density,tmp = INT_MIN;
    MDD = INT_MIN;
    Layout_Patterns S;

    //[1]. calculate mask density value in Window level.
    for(int i = 0; i < packing_list.size(); i++)
    {
        tmp = INT_MIN;
        for(int j = 0; j < mask_num; j++)
        {
            // t1,t2 = total area of layout patterns which mask id is 'j'. 
            // 'i' is the window id.
            t1 = get_mask_area(0,j);
            t2 = get_mask_area(i,j);

            MDD = max(abs(t1 - t2), MDD);

            // calculate the density difference. ex:t1 = 1.5, t2 = 1! then density = (1.5 - 1) / 1 = 0.5
            if(t1 == 0 || t2 == 0)
                density = 0;
            else
                density = t1 >= t2 ? (t1 / t2 - 1) : (t2 / t1 - 1);
            if(tmp < density)
                tmp = density;
        }
        Gmd += (tmp - mask_density_difference) < 0 ? 0 : (tmp - mask_density_difference);        
    }
    //[2]. calculate mask density cost in Bin level.
    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
           continue;

        vector<Node*> nodes = wbnodes[i].cbtree->allnodes();
        for(int j = 0; j < nodes.size(); j++)
        {
            for(int k = 0; k < modules_info[nodes[j]->id].patterns.size(); k++)
            {
                S.push_back(modules_info[nodes[j]->id].patterns[k]);
            }
        }
    }
    t1 = 0;
    for(int i = 0; i < S.size(); i++)
    {
        if(!isIsolated(S[i]))
        {
            //[1]. create a BIN
            Rect bin;
            x = S[i].layout.right - (S[i].layout.right - S[i].layout.left) / 2; 
            y = S[i].layout.top - (S[i].layout.top - S[i].layout.bottom) / 2; 
            bin.left = x - bin_w / 2;
            bin.right = x + bin_w / 2;
            bin.bottom = y - bin_h / 2;
            bin.top = y + bin_h / 2;
            //[2]. mask density difference.
            d= get_mask_density_in_bin(bin, S[i]);
            t2 = get_mask_density_in_bin(bin, S[i]);
            
            if(t1 == 0)
                t1 = t2;
            if(t1 == 0 || t2 == 0)
                tmp = 0;
            else
                tmp = t1 >= t2 ? (t1-t2)/t2 : (t2-t1)/t1;
            Lmd += (tmp - mask_density_difference) < 0 ? 0 : (tmp - mask_density_difference);
        }
    }

    return (Mcft + Gmd + Lmd)*1000;
}

/* Calculate the the number of patterns which violate the mask conflict constraint. */
double WB_Tree::calculate_Ncft(int mod_id, int pattern_id)
{
    int Ncft = 0;
    int mask_id = modules_info[mod_id].patterns[pattern_id].mask_id;
    double dis,x1,y1,x2,y2,left,right,top,bottom,left_t,right_t,top_t,bottom_t;
    //[1]. Find the wbnode with module id and get the boundary of the pattern with pattern_id
    WBNode *p_node = find_wbnode(mod_id);
    left = modules_info[mod_id].patterns[pattern_id].layout.left;
    right = modules_info[mod_id].patterns[pattern_id].layout.right;
    top = modules_info[mod_id].patterns[pattern_id].layout.top;
    bottom = modules_info[mod_id].patterns[pattern_id].layout.bottom;
    
    // Traverse all modules.
    for(int i = 0; i < modules_N; i++)
    {
        if(mod_id == i || modules[i].Dfin == 0)
            continue;

        for(int j = 0; j < modules_info[i].patterns.size(); j++)
        {
            if(modules_info[i].patterns[j].internal || mask_id != modules_info[i].patterns[j].mask_id)
                continue;
            // get the opportunity patterns' boundary
            WBNode *t_node = find_wbnode(i);
            left_t = modules_info[i].patterns[j].layout.left;
            right_t = modules_info[i].patterns[j].layout.right;
            top_t = modules_info[i].patterns[j].layout.top;
            bottom_t = modules_info[i].patterns[j].layout.bottom;

            // calculate the minium distance between opportunity pattern and self pattern
            x1 = min(min(abs(left_t - left) , abs(left_t - right)), min(abs(right_t - left) , abs(right_t - right))); 
            y1 = min(min(abs(top_t - top) , abs(top_t - bottom)), min(abs(bottom_t - top) , abs(bottom_t - bottom)));
            dis = sqrt(pow(x1,2) + pow(y1,2));
            if(dis < minimum_distance) // if the distance is smaller than minimum_distance
                Ncft++;
        }

    }
    return Ncft;
}

double WB_Tree::calcMaskConfictCost()
{
    double Ncft = 0,Ns = 0;
    for(int i = 0; i < modules_N; i++)
    {
        if(modules[i].Dfin == 0)
            continue;

        // calculate Ncft
        for(int j = 0; j < modules_info[i].patterns.size(); j++){
            if(modules_info[i].patterns[j].internal)
                continue;
            Ncft += calculate_Ncft(i, j);
        }

        // calculate Ns
        Ns += modules_info[i].patterns.size();
    }

    Ncft = Ncft / 2;

    return (double)Ncft / (double)pow(Ns,2) * 100000;
}

double WB_Tree::calcOutBoundArea()
{
    double O = 0;
    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
            continue;
        if(wbnodes[i].cbtree->getWidth() > win_w && wbnodes[i].cbtree->getHeight() > win_h)
        {
            O += (wbnodes[i].cbtree->getArea() - win_w * win_h);
        }
        if(wbnodes[i].cbtree->getWidth() > win_w && wbnodes[i].cbtree->getHeight() <= win_h)
        {
            O += (wbnodes[i].cbtree->getWidth() - win_w) * wbnodes[i].cbtree->getHeight();
        }
        if(wbnodes[i].cbtree->getHeight() > win_h && wbnodes[i].cbtree->getWidth() <= win_w)
        {
            O += (wbnodes[i].cbtree->getHeight() - win_h) * wbnodes[i].cbtree->getWidth();
        }
    }
    return O * 1e-3;
}

void WB_Tree::scaleIOPad()
{
    float px = width / float(root_module.width);
    float py = height / float(root_module.height);

    for (int i = 0; i < root_module.pins.size(); i++)
    {
        Pin &p = root_module.pins[i];
        p.ax = int(px * p.x);
        p.ay = int(py * p.y);
    }
}

double WB_Tree::calcWireLength()
{
    scaleIOPad();
    WireLength = 0;
    // compute absolute position
    for (int i = 0; i < modules.size(); i++)
    {
        int mx = modules_info[i].x;
        int my = modules_info[i].y;
        bool rotate = modules_info[i].rotate;
        int w = modules[i].width;

        for (int j = 0; j < modules[i].pins.size(); j++)
        {
            Pin &p = modules[i].pins[j];
            if (!rotate)
            {
                p.ax = p.x + mx;
                p.ay = p.y + my;
            }
            else
            { // Y' = W - X, X' = Y
                p.ax = p.y + mx;
                p.ay = (w - p.x) + my;
            }
        }
    }
    for (int i = 0; i < network.size(); i++)
    {
        int max_x = INT_MIN;
        int max_y = INT_MIN;
        int min_x = INT_MAX;
        int min_y = INT_MAX;
        assert(network[i].size() > 0);
        for (int j = 0; j < network[i].size(); j++)
        {
            Pin &p = *network[i][j];
            max_x = max(max_x, p.ax);
            max_y = max(max_y, p.ay);
            min_x = min(min_x, p.ax);
            min_y = min(min_y, p.ay);
        }
        WireLength += (max_x - min_x) + (max_y - min_y);
    }

    return WireLength;
}

//--------------------------------------------------------------------------------
/*           Base Actions           */
//--------------------------------------------------------------------------------

void WB_Tree::normalize_cost(int t)
{
    double norm_area=0,norm_wire=0,pre_cost = INT_MAX;
    for(int i=0; i < t; i++){
        perturb();
        calculate_cost();
        //packing();
        if(pre_cost > cost)
        {
            pre_cost = cost;

            keep_sol(last_sol);
        }
        norm_area += Area;
        norm_wire += WireLength;
    }

    norm_area /= t;
    norm_wire /= t;
    printf("normalize area=%.0f, wire=%.0f\n", norm_area, norm_wire);
}

// find random wbnode that has a CB-Tree
WBNode* WB_Tree::find_window_random()
{
    int i, c = NIL;

    for (i = 0; i < wbnodes.size(); i++)
    {
        if (wbnodes[i].cbtree == nullptr)
        {
            c++;
            break;
        }
    }

    if (c == NIL)
        return nullptr; // there is no CB-Tree.
    do
    {
        i = rand() % wbnodes.size();
    } while (wbnodes[i].cbtree != nullptr);

    return &wbnodes[i];
}

// Find wbnode with module id
WBNode* WB_Tree::find_wbnode(int mod_id)
{
    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
            continue;
        vector<Node*> nodes =wbnodes[i].cbtree->allnodes();
        for(int j = 0; j < nodes.size(); j++)
        {
            if(nodes[j]->id == mod_id)
                return &wbnodes[i];
        }
    }
    return nullptr;
}

// Find wbnode with CB_tree
WBNode* WB_Tree::find_wbnode(CB_Tree* cbtree)
{
    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
            continue;
        if(wbnodes[i].cbtree == cbtree)
            return &wbnodes[i];
    }

    return nullptr;
}

// Find the module id with the module name
int WB_Tree::find_mod(char* module_name)
{
    int i;
    for(i = 0; i < modules_N; i++)
    {
        if(strcmp(modules[i].name, module_name) == 0)
            return i;
    }
    error("Can not find module which name is %s.",module_name);
    return -1;
}
// split the last character of the string.
char* tail(char *str)
{
    str[strlen(str) - 1] = 0;
    return str;
}

// Read modules from input file [apte]
void WB_Tree::read_module_info()
{
    fs.open(filename);
    if(fs.fail())
        error("unable to open file: %s",filename);

    bool final=false;
    Module dummy_mod;
    for(int i=0; !fs.eof(); i++){
        // modules
        modules.push_back(dummy_mod);	// new module
        Module &mod = modules.back();
        mod.id = i;
        mod.pins.clear();
        mod.Dbot = mod.Dfin = mod.Dtop = 0;

        fs >> t1 >> t2;
        tail(t2);			// remove ";"
        strcpy(mod.name,t2);

        fs >> t1 >> t2;
        if(!strcmp(t2,"PARENT;"))
            final= true;

        // dimension
        read_dimension(mod);
        read_IO_list(mod,final);

        // network
        if(final){
            read_network();
            break;
        }
    }

    root_module = modules.back();
    modules.pop_back();		// exclude the parent module
    modules_N = modules.size();
    modules_info.resize(modules_N);
    modules.resize(modules_N);

    create_network();

    TotalArea = 0;
    for(int i=0; i < modules_N; i++)
        TotalArea += modules[i].area;
    for(int i = 0; i < constraints.preplaced.size(); i++)
    {
        TotalArea += ( constraints.preplaced[i].rx - constraints.preplaced[i].x ) * ( constraints.preplaced[i].ry - constraints.preplaced[i].y );
    }

    fs.close();
}

// Set the module's size and width ad height and area
void WB_Tree::read_dimension(Module &mod)
{
    fs >> t1;
    int min_x=INT_MAX,min_y=INT_MAX,max_x=INT_MIN,max_y=INT_MIN;
    int tx,ty;
    for(int i=0; i < 4;i++){
        fs >> tx >> ty;
        min_x=min(min_x,tx); max_x=max(max_x,tx);
        min_y=min(min_y,ty); max_y=max(max_y,ty);
    }

    mod.x      = min_x;
    mod.y      = min_y;
    mod.width  = max_x - min_x;
    mod.height = max_y - min_y;
    mod.area   = mod.width * mod.height;
    fs >> t1 >> t2;
}

// Read the Pin informations from the input file.
void WB_Tree::read_IO_list(Module &mod,bool parent=false)
{
    // IO list
    while(!fs.eof()){
        Pin p;
        fs.getline(line,100);
        if(strlen(line)==0) continue;
        sscanf(line,"%s %*s %d %d",t1,&p.x,&p.y);

        if(!strcmp(t1,"ENDIOLIST;"))
            break;

        if(parent){ // IO pad is network
            // make unique net id
            net_table.insert(make_pair(string(t1),net_table.size()));
            p.net = net_table[t1];
        }

        p.mod = mod.id;
        p.x -= mod.x;  p.y -= mod.y;	// shift to origin

        mod.pins.push_back(p);
    }
    fs.getline(line,100);
}

// Create the network with Pins
void WB_Tree::create_network(){
    network.resize(net_table.size());

    for(int i=0; i < modules_N; i++){
        for(int j=0; j < modules[i].pins.size(); j++){
            Pin &p = modules[i].pins[j];
            network[p.net].push_back(&p);
        }
    }

    for(int j=0; j < root_module.pins.size(); j++){
        Pin &p = root_module.pins[j];
        network[p.net].push_back(&p);
    }

    connection.resize(modules_N+1);
    for(int i=0; i < modules_N+1; i++){
        connection[i].resize(modules_N+1);
        fill(connection[i].begin(), connection[i].end(), 0);
    }

    for(int i=0; i < network.size(); i++){
        for(int j=0; j < network[i].size()-1; j++){
            int p= network[i][j]->mod;
            for(int k=j+1 ; k < network[i].size(); k++){
                int q= network[i][k]->mod;
                connection[p][q]++;
                connection[q][p]++;
            }
        }
    }
}

// Read the conncection information from the input file.
void WB_Tree::read_network(){
    while(!fs.eof()){
        bool end=false;
        int n=0;
        fs >> t1 >> t2;
        if(!strcmp(t1,"ENDNETWORK;"))
            break;
        // determine which module interconnection by name
        int m_id;
        for(m_id=0; m_id < modules.size(); m_id++)
            if(!strcmp(modules[m_id].name,t2))
                break;
        if(m_id== modules.size())
            error("can't find suitable module name!");

        while(!fs.eof()){
            fs >> t1;
            if(t1[strlen(t1)-1]==';'){
                tail(t1);
                end=true;
            }

            // make unique net id
            net_table.insert(make_pair(string(t1),net_table.size()));
            modules[m_id].pins[n++].net = net_table[t1];
            if(end) break;
        }
    }
}

// Read the geometrical constraints from the input file [*.constraint]
void WB_Tree::read_constraint()
{
    char constraint_file[80] = "";
    strcpy(constraint_file,filename);
    strcat(constraint_file,".constraint");
    fs.open(constraint_file);
    if (fs.fail())
        error("unable to open file: %s", constraint_file);

    while (!fs.eof())
    {
        fs >> t1;
        if (strcmp(t1, "PROXIMITY") == 0)
            read_proximity_constraint();
        else if(strcmp(t1, "FIXED_BOUNDARY") == 0)
            read_fixed_boundary_constraint();
        else if(strcmp(t1, "BOUNDARY") == 0)
            read_boundary_constraint();
        else if(strcmp(t1, "MINIMUM_SEPERATION") == 0)
            read_minimum_seperation();
        else if(strcmp(t1, "SYMMETRY") == 0)
            read_symmetry_constraint();
        else if(strcmp(t1, "PREPLACED") == 0)
            read_preplaced_constraints();
    }

    fs.close();

}

void WB_Tree::read_preplaced_constraints()
{
    while(!fs.eof()){
        fs>>t1;
        if (strcmp(t1, "END_PREPLACED") == 0)
            break;
        PREPLACED p;
        strcpy(p.mod,t1);
        fs>>t1>>t2;
        p.x = atoi(t1);
        p.y = atoi(t2);
        fs>>t1>>t2;
        tail(t2);
        p.rx = atoi(t1);
        p.ry = atoi(t2);
        constraints.preplaced.push_back(p);
    }   
}

void WB_Tree::read_symmetry_constraint()
{
    char *token;
    bool f = false;
    SYMMETRY sym;
    while(!fs.eof())
    {
        fs>>t1;    
        //cout<<t1<<endl;
        if (strcmp(t1, "END_SYMMETRY") == 0)
            break;

        if (t1[0] == '[')
        {
            token = strtok(t1, "[,]");
            f = true;
            while (token != NULL)
            {
                //cout<<token<<" : "<<f<<endl;
                if(f){
                    sym.sequance_pair1.push_back(find_mod(token));
                    f = false;
                }else{
                    sym.sequance_pair2.push_back(find_mod(token));
                    f = true;
                }
                token = strtok(NULL, "[,]");
            }
        }else if(strcmp(t1,"VERTICAL") == 0)
        {
            fs>>t2;
            if (t2[strlen(t2) - 1] == ';')
            {
                tail(t2);
            }
            sym.axis = atoi(t2);
            sym.is_vertical = true;
            constraints.symmetry.push_back(sym);
            sym = {};
        }else if(strcmp(t1,"HORIZONTAL") == 0){
            fs>>t2;
            if (t2[strlen(t2) - 1] == ';')
            {
                tail(t2);
            }
            sym.axis = atoi(t2);
            sym.is_vertical = false;
            constraints.symmetry.push_back(sym);
            sym = {};
        }else
        {
            sym.self_symmetry_mods.push_back(find_mod(t1));
        }
    }
    // for(int i = 0; i < constraints.symmetry.size(); i++)
    // {
    //     cout<<"symmetry_pair1 : "<<constraints.symmetry[i].sequance_pair1.size()<<endl;
    //     cout<<"symmetry_pair2 : "<<constraints.symmetry[i].sequance_pair1.size()<<endl;
    //     cout<<"self_symmetry : "<<constraints.symmetry[i].self_symmetry_mods.size()<<endl;
    // }
}

void WB_Tree::read_minimum_seperation()
{
    while (!fs.eof())
    {
        MINIMUM_SEPERATION min;
        min.mod = NIL;
        min.dis = NIL;
        fs >> t1;
        if (strcmp(t1, "END_MINIMUM_SEPERATION") == 0)
            break;
        fs >> t2;
        tail(t2);
        min.mod = find_mod(t1);
        min.dis = atoi(t2);
        constraints.min_sep.push_back(min);
    }
}

void WB_Tree::read_boundary_constraint()
{
    while (!fs.eof())
    {
        fs >> t1;
        if (strcmp(t1, "END_BOUNDARY") == 0)
            break;
        tail(t1);
        constraints.boundary.push_back(find_mod(t1));
    }
}

void WB_Tree::read_fixed_boundary_constraint()
{
    while (!fs.eof())
    {
        FIXED_BOUNDARY fb;
        fs >> t1;
        if (strcmp(t1, "END_FIXED_BOUNDARY") == 0)
            break;

        fs >> t2;

        if(strcmp(t2, "1") == 0) // rotate
            fb.rotate = true;
        else
            fb.rotate = false;

        fs >> t2;
        tail(t2);

        if(strcmp(t2, "1") == 0)
            fb.flip = true;
        else
            fb.flip = false;

        fb.mod = find_mod(t1);
        constraints.fixed_boundary.push_back(fb);
    }
}

void WB_Tree::read_proximity_constraint()
{
    while (!fs.eof())
    {
        fs >> t1;
        if (strcmp(t1, "END_PROXIMITY") == 0)
            break;
        if (t1[strlen(t1) - 1] == ';')
        {
            tail(t1);
        }
        constraints.proximity.push_back(find_mod(t1));
    }
}

// Read the Fin-Fet design constraints from input file [*.design]
void WB_Tree::read_design_rules()
{
    int i;
    char design_file[80] = "";
    strcpy(design_file,filename);
    strcat(design_file,".design");

    //[1]. Open design_rules file.
    fs.open(design_file);
    if (fs.fail())
        error("unable to open file: %s", design_file);

    //[2]. Read information in the file.
    fs>>t1;
    if(strcmp(t1,"LAYOUT;") == 0)
    {
        while(!fs.eof())
        {
            fs>>t1;
            if(strcmp(t1,"END_LAYOUT;") == 0)
                break;
            if(strcmp(t1,"MODULE") == 0)
            {
                fs>>t2;
                tail(t2);
                i = find_mod(t2);
            }

            fs>>t1;
            if(strcmp(t1,"PATTERNS;") == 0)
            {
                fs>>t1;
                Layout_Patterns temp;
                while(!fs.eof())
                {
                    if(strcmp(t1,"END_PATTERNS;") == 0)
                        break;
                    Layout_Pattern p;
                    fs>>t2;
                    p.layout.left = atoi(t2);
                    fs>>t2;
                    p.layout.bottom = atoi(t2);
                    fs>>t2;
                    p.layout.right = atoi(t2);
                    fs>>t2;
                    tail(t2);
                    p.layout.top = atoi(t2);
                    temp.push_back(p);
                    fs>>t1;
                }
                layout_patterns.insert(make_pair(i,temp));
                modules_info[i].patterns.resize(temp.size());
            }
            fs>>t1;
            if(strcmp(t1,"FIN") == 0)
            {
                fs>>t1>>t2;
                tail(t2);
                modules[i].Dfin = atoi(t1);
                Dfin = modules[i].Dfin;
                modules[i].Dbot = atoi(t2);
                modules[i].Dtop = (modules[i].height - modules[i].Dbot) % modules[i].Dfin;
            }
            fs>>t1;
        }
    }
    fs>>t1;
    if(strcmp(t1,"DESIGN_RULES;") == 0)
    {
        //[1]. MINIMUM_DISTANCE value.
        fs>>t1>>t2;
        tail(t2);
        minimum_distance = atoi(t2);

        //[2]. MASK_NUM value.
        fs>>t1>>t2;
        tail(t2);
        mask_num = atoi(t2);

        //[3]. WINDOW_SIZE value value
        fs>>t1;
        fs>>t1>>t2;
        tail(t2);
        win_w = atoi(t1);
        win_h = atoi(t2);

        //[4]. BIN_SIZE value value
        fs>>t1;
        fs>>t1>>t2;
        tail(t2);
        bin_w = atoi(t1);
        bin_h = atoi(t2);

        //[5]. MAXIMUM DENSITY DIFFERENCE
        fs>>t1>>t2;
        mask_density_difference = atof(t2);
    }

    fs.close();
}

// After the getting the result, it outputs the result information into the console and output file *.res.
void WB_Tree::list_information(){

    //  string info = filename + ".info";
    char info[80];
    strcpy(info,filename);
    strcat(info,".info");
    ofstream of(info);

    of << modules_N << " " << width << " " << height << endl;
    for(int i=0; i < modules_N; i++){
        of << modules_info[i].x  << " " << modules_info[i].rx  << " ";
        of << modules_info[i].y << " " << modules_info[i].ry << endl;
    }
    of << endl;

    calcWireLength();
    int x,y,rx,ry;
    for(int i=0; i < network.size(); i++){
        assert(network[i].size()>0);
        x = network[i][0]->ax;
        y = network[i][0]->ay;

        for(int j=1; j < network[i].size(); j++){
            rx = network[i][j]->ax;
            ry = network[i][j]->ay;
            of << x << " " << y << " " << rx << " " << ry << endl;
            x = rx, y = ry;
        }
    }

    cout << "Num of Module      = " << modules_N + constraints.preplaced.size() << endl;
    cout << "Wire Length        = " << calcWireLength()*1e-3 << endl;
    cout << "Height             = " << height*1e-3 << endl;
    cout << "Width              = " << width*1e-3 << endl;
    cout << "Area               = " << Area*1e-6 << endl;
    cout << "Out of bound Area  = " << calcOutBoundArea()*1e-3<<endl;
    cout << "Total Area         = " << TotalArea*1e-6 << endl;
    cout << "Total cost         = " << cost<<endl;
    cout << "MDD         = " << MDD<<endl;
 	cout<< "     ----  "<<120000*1e-6<<endl;
    printf( "Dead Space     = %.2f\n", get_dead_space());

}

// calculate the white space of the WB-Tree. 
double WB_Tree::get_dead_space()
{
    double ds = 0;
    for(int i = 0; i < wbnodes.size(); i++)
    {
        if(wbnodes[i].cbtree == nullptr)
        {
            ds += win_h * win_w;
        }
        else
        {
            ds += win_h * win_w - (wbnodes[i].cbtree->getWidth() * wbnodes[i].cbtree->getHeight());
        }
    }
    return abs(ds*1e-6);
}

//  Show the wb-tree's structure into the console.
void WB_Tree::show_tree()
{
    for(int i = 0; i < wbnodes.size(); i++)
    {
        cout<<"row:"<<wbnodes[i].row<<", col:"<<wbnodes[i].col<<endl;
        if(wbnodes[i].cbtree != nullptr)
            wbnodes[i].cbtree->show_tree();
    }
//    wbnodes[1].cbtree->show_modules();
}

// Create the Matlab execution file.
void WB_Tree::output()
{
    int x1,y1,x2,y2;
    FILE *fs= fopen("output.m","w");
    //[1]. Draw the Preplaced module.
    for(int i = 0; i < constraints.preplaced.size(); i++)
    {
        x1 = constraints.preplaced[i].x;
        y1 = constraints.preplaced[i].y;
        x2 = constraints.preplaced[i].rx;
        y2 = constraints.preplaced[i].ry;

        if(x1 > x2)
            swap(x1,x2);
        if(y1 > y2)
            swap(y1,y2);

        fprintf(fs,"%%PPM%d \n",i+1);
        fprintf(fs,"x1=%d; \n",x1);
        fprintf(fs,"x2=%d; \n",x2);
        fprintf(fs,"y1=%d; \n",y1);
        fprintf(fs,"y2=%d; \n",y2);
        fprintf(fs,"rectangle('Position',[x1,y1,x2-x1,y2-y1],'FaceColor',[.3 .6 .9],'EdgeColor','b','LineWidth',1);\n");
        fprintf(fs,"str = 'PPM - %s'; \n nstr = strrep(str,'_',' '); \n text(x1+5,y2-((y2-y1)/4),nstr);",constraints.preplaced[i].mod);
    }
    //[2]. Draw the normal modules and layout patterns.
    for(int i = 0; i < modules_info.size(); i++)
    {
        x1 = modules_info[i].x;
        x2 = modules_info[i].rx;
        y1 = modules_info[i].y;
        y2 = modules_info[i].ry;
        if(x1 > x2)
            swap(x1,x2);
        if(y1 > y2)
            swap(y1,y2);

        fprintf(fs,"%%module%d \n",i+1);
        fprintf(fs,"x1=%d; \n",x1);
        fprintf(fs,"x2=%d; \n",x2);
        fprintf(fs,"y1=%d; \n",y1);
        fprintf(fs,"y2=%d; \n",y2);
        fprintf(fs,"rectangle('Position',[x1,y1,x2-x1,y2-y1],'FaceColor',[.6 .6 .6],'EdgeColor','b','LineWidth',1);\n");
        fprintf(fs,"str = '%s'; \n nstr = strrep(str,'_',' '); \n text(x1+5,y2-((y2-y1)/4),nstr);",modules[i].name);

        if(modules[i].Dfin == 0)
            continue;

        for(int j = 0; j < modules_info[i].patterns.size(); j++)
        {
            float r = (float)((8 + 3 * modules_info[i].patterns[j].mask_id) % 10) / 10;
            float g = (float)((4 + 3 * modules_info[i].patterns[j].mask_id) % 10) / 10;
            float b = (float)((1 + 3 * modules_info[i].patterns[j].mask_id) % 10) / 10;

            x1 = modules_info[i].patterns[j].layout.left;
            x2 = modules_info[i].patterns[j].layout.right;
            y1 = modules_info[i].patterns[j].layout.bottom;
            y2 = modules_info[i].patterns[j].layout.top;
            if(x1 > x2)
                swap(x1,x2);
            if(y1 > y2)
                swap(y1,y2);

            fprintf(fs,"x1=%d; \n",x1);
            fprintf(fs,"x2=%d; \n",x2);
            fprintf(fs,"y1=%d; \n",y1);
            fprintf(fs,"y2=%d; \n",y2);
            fprintf(fs,"rectangle('Position',[x1,y1,x2-x1,y2-y1],'FaceColor',[%.1f %.1f %.1f],'EdgeColor','r','LineWidth',1);\n", r, g, b);
        }
    }

    //[3]. Draw the fin lines.
    for(int i = 0; i < modules_info.size(); i++)
    {
        if(modules_info[i].Dfin == 0)
            continue;
        for(int d = modules_info[i].Dbot + modules_info[i].y; d < modules_info[i].ry; d+=300)
        {
            fprintf(fs,"%%line%d \n",d);
            fprintf(fs, "line([%d %d], [%d %d])\n",modules_info[i].x,modules_info[i].rx,d,d);
        }
    }
    
    //[4]. Draw the Window regions.
    for(int i = 0; i < wbnodes.size(); i++)
    {
        fprintf(fs,"%%region \n");
        fprintf(fs,"x1=%d; \n",wbnodes[i].col * win_w);
        fprintf(fs,"x2=%d; \n",(wbnodes[i].col+1) * win_w);
        fprintf(fs,"y1=%d; \n",wbnodes[i].row * win_h);
        fprintf(fs,"y2=%d; \n",(wbnodes[i].row+1) * win_h);
        fprintf(fs,"rectangle('Position',[x1,y1,x2-x1,y2-y1],'FaceColor','none','EdgeColor',[%f %f %f],'LineWidth',1,'LineStyle','--');\n",0.0,0.0,0.0);
    }

    fclose(fs);
}

//------------------------------------------------------------------------------
/*              Solution Management                                      */
//-------------------------------------------------------------------------------

// Keep the current the WB-Tree Structure as the Solution.
void WB_Tree::keep_sol(Solution &sol)
{
    sol.wbnodes = wbnodes;
    // copy new
    for (int i = 0; i < wbnodes.size(); i++)
    {
        if (wbnodes[i].cbtree != nullptr)
        {
            auto nodes = new vector<Node>();
            wbnodes[i].cbtree->copy_tree(*nodes);       // get the nodes from the CB-Tree
            sol.wbnodes[i].cbtree = (CB_Tree *)nodes; // save the nodes as the CB-Tree class type.
        }
        else
        {
            sol.wbnodes[i].cbtree = nullptr;
        }
    }
    sol.cost = cost;
}

// Recover the Solution's WB-Tree structure.
void WB_Tree::recover(Solution &sol)
{
    //[1]. Safe delete the current WB-Tree
    for (int i = 0; i < packing_list.size(); i++)
    {
        packing_list[i]->destroy();
        SAFE_DELETE(packing_list[i]);
    }
    packing_list.clear();
    //[2]. Set the WB-Tree of the solution.
    wbnodes = sol.wbnodes;
    for (int i = 0; i < sol.wbnodes.size(); i++)
    {
        if (sol.wbnodes[i].cbtree)
        {
            // Solution's CB-Trees are vector<Node*>, not a CB-Tree class. So we have to create the CB-Tree and set the nodes.
            wbnodes[i].cbtree = new CB_Tree(alpha);
            vector<Node> *t = (vector<Node> *)sol.wbnodes[i].cbtree;  // convert the Solution's CB-Tree class type to vector<Node>.
            auto tree = new vector<Node>();
            copy_tree(t, tree);                 // copy the nodes.
            init_cbtree(*wbnodes[i].cbtree);    // init the CB-Tree
            wbnodes[i].cbtree->preplaced = get_preplaced_module(&wbnodes[i]); // set the Preplaced module.
            
            wbnodes[i].cbtree->nodes_root = tree->data();   // set the root node of the CB-Tree
            wbnodes[i].cbtree->init_with_out_node();        // Set basic value of the CB-Tree

            packing_list.push_back(wbnodes[i].cbtree);      // add it to the Packing list
        }else
        {
            wbnodes[i].cbtree = nullptr;
        }
    }

    cost = sol.cost;
}

//  Copy tree from the nodes array.
void WB_Tree::copy_tree(vector<Node> *tree_o, vector<Node> *tree)
{
    // copy tree vector.
    for(int i = 0; i < tree_o->size(); i++)
    {
        Node* node = (Node*) new unsigned char[sizeof(Node)];
        node->id = tree_o->at(i).id;
        node->flip = tree_o->at(i).flip;
        node->rotate = tree_o->at(i).rotate;
        node->parent = node->left = node->right = nullptr;
        tree->push_back(*node);
    }

    for(int i = 0; i < tree_o->size(); i++)
    {
        for(int j = 0; j < tree_o->size(); j++)
        {
            // j is i's parent.
            if(&tree_o->at(j) == tree_o->at(i).parent)
            {
                tree->at(i).parent = &tree->at(j);
            }
            // j is i's left child
            if(&tree_o->at(j) == tree_o->at(i).left)
            {
                tree->at(i).left = &tree->at(j);
            }
            // j is i's right child
            if(&tree_o->at(j) == tree_o->at(i).right)
            {
                tree->at(i).right = &tree->at(j);
            }
        }
    }
}
