//---------------------------------------------------------------------------
#include <stack>
#include <algorithm>
#include "cbtree.h"
#include <iostream>
#include <climits>
#include <string.h>
#include <math.h>
#include <time.h>
// Safe delete. manages the memory effectness.
#define SAFE_DELETE(p) { \
    if(p==NULL) { \
    std::string str="\ndelete on nullptr: "; \
    str+=__FILE__; str+="//"; str+=std::to_string(__LINE__);str+="\n"; \
    printf("%s",str.c_str()); \
    throw std::runtime_error("DELETE ON NULLPTR"); \
    } \
    else{ \
    p=NULL; \
    } \
}

using namespace std;
//--------------------[not used in WB-Tree. It used in CB-Tree's optimization.]--------------------------------------
float rotate_rate = 0.3;    // random rotate rate.
float swap_rate = 0.5;      // randm swap rate.

//---------------------------------------------------------------------------
//   Initialization
//---------------------------------------------------------------------------

void CB_Tree::clear(){
    // initial contour value
    FPlan::clear();
}

// init the CB-Tree.
void CB_Tree::init()
{
    vector<int> inds;
    for(int i=0; i < modules_N; i++){
        inds.push_back(i);
    }
    init_with_node_indices(inds);
}

// calculate the sum of the modules' area
void CB_Tree::calc_total_area()
{
    TotalArea = 0;
    auto nodes = allnodes();
    for(int i=0; i<nodes.size(); i++)
    {
        TotalArea += modules[nodes[i]->id].area;  // sum of the modules' area
    }
    for(int i = 0; i < preplaced.size(); i++)
    {
        TotalArea += (preplaced[i].rx - preplaced[i].x) * (preplaced[i].ry - preplaced[i].y); // sum of the preplaced modules's area.
    }
}

// init the CB-Tree. it has modules already.
void CB_Tree::init_with_out_node()
{
    clear();

    TotalArea = 0;
    for(int i=0; i < modules_N; i++){
        TotalArea += modules[i].area;    // calculate the sum of the modules' area.
    }

    modules_info.resize(modules_N);

    make_tiles();                       // make tiles map based on modules.

    auto nodes = allnodes();            
    nodes_N = nodes.size();             // set the nodes' number

    normalize_cost(1);                  // placed the module.
}

// Make the tiles' map.
void CB_Tree::make_tiles()
{
    for(int i = 0; i < modules_N; i++)
    {
        Tiles tiles;
        tiles.resize(3);
        tile_map.insert(make_pair(i,tiles));        // insert the tiles with the module's id
        modules_info[i].patterns.resize(layout_patterns[i].size()); // prepare the layout patterns in every module.
    }

    for(int i = 0; i < preplaced.size(); i++)
    {
        Tiles tiles;
        tiles.resize(3);
        tile_map.insert(make_pair(i + modules_N,tiles));    // insert the tils of the preplaced module
        preplaced[i].id = i + modules_N;
    }
}

// Initialize the tiles. set Nil.
void CB_Tree::init_tiles()
{
    for(int i = 0; i < modules_N; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            tile_map[i][j].set_Nil();
        }
    }

    // But preplaced module has already fixed position, so we set the value in the tiles of the Preplaced modules.
    for(int i = 0; i < preplaced.size(); i++)
    {
        for(int j = 0; j < 3; j++)
        {
            tile_map[preplaced[i].id][0].set_Val(preplaced[i].x, preplaced[i].y, preplaced[i].rx, preplaced[i].ry, true);
            tile_map[preplaced[i].id][1].set_Val(preplaced[i].x, preplaced[i].ry, preplaced[i].rx, estimate_height, false);
            tile_map[preplaced[i].id][2].set_Val(preplaced[i].rx, 0, estimate_width, estimate_height, false);
        }
    }
}

// Init the CB-Tree with nodes. it doesn't have nodes.
void CB_Tree::init_with_node_indices(vector<int> indices)
{
    TotalArea = 0;
    vector<Node*> nodes;
    //[1]. Determine the module info.
    modules_info.resize(modules_N);

    make_tiles();

    // set every id of nodes
    for(int i=0; i<indices.size(); i++)
    {
        Node* node = (Node*) new unsigned char[sizeof(Node)];
        memset(node, 0, sizeof(Node));
        nodes.push_back(node);
        node = nodes.back();
        node->id = indices[i];
        node->parent = nodes[(i+1)/2-1];
        TotalArea += modules[indices[i]].area;
    }

    // set left&right child
    for(int i=0; i<indices.size(); i++)
    {
        nodes[i]->left   = (2*i+1 < indices.size() ? nodes[2*i+1] : nullptr);
        nodes[i]->right  = (2*i+2 < indices.size() ? nodes[2*i+2] : nullptr);
    }

    // set root node
    nodes_root = nodes[0];
    nodes[0]->parent = nullptr;
    nodes_N = nodes.size();

    clear();

    // make initial width and height of B*-tree.
    estimate_width = estimate_height = sqrt(TotalArea);

    normalize_cost(10);
}



//---------------------------------------------------------------------------
//   Placement modules
//---------------------------------------------------------------------------

//CB-Tree's packing.
void CB_Tree::packing(){
    stack<Node*> S;

    Node* p = nodes_root;

    init_tiles();

    place_module(p);  // place root module, determine the position of the root module.
    place_patterns(p);  // determine the position of the layout patterns according to the module's position.(Because the layout patterns are in the module).
    Node *n = p;
    if(n->right != nullptr)      S.push(n->right);
    if(n->left  != nullptr)      S.push(n->left);
    // inorder traverse
    while(!S.empty()){
        p = S.top();
        S.pop();
        Node *n = p;

        assert(n->parent != nullptr);
        place_module(p);    // place the module.
        place_patterns(p);  // place the layout patterns in the module.
        if(n->right != nullptr)      S.push(n->right);
        if(n->left  != nullptr)      S.push(n->left);
    }

    // compute Width, Height
    double max_x=-1,max_y=-1;
    auto nodes = allnodes();
    for(int i = 0; i < nodes.size(); i++){
        max_x = max(max_x,double(modules_info[nodes[i]->id].rx));
        max_y = max(max_y,double(modules_info[nodes[i]->id].ry));
    }

    Width  = max_x;
    Height = max_y;
    Area   = Height*Width;

    calc_total_area();
}

// Place the module.
void CB_Tree::place_module(Node* mod)
{
    int temp,max_y = INT_MIN,curY,newY;
    int pid,Dbot,Dtop,Dfin;
    int x,y;
    Module_Info &mod_mf = modules_info[mod->id];

    //[1]. Set the Fin information.
    mod_mf.Dbot = modules[mod->id].Dbot;
    mod_mf.Dfin = modules[mod->id].Dfin;
    mod_mf.Dtop = modules[mod->id].Dtop;
    mod_mf.rotate       = mod->rotate;
    mod_mf.flip         = mod->flip;

    //[2]. Correct the rotaion and flip information according to the fixed_boundary.
    for(int i = 0; i < fb.size(); i++)
    {
        if(mod->id == fb[i].mod)
        {
            mod_mf.rotate       = fb[i].rotate;
            mod_mf.flip         = fb[i].flip;
        }
    }

    //[3]. Get the module's width and height.
    int w =  modules[mod->id].width;
    int h =  modules[mod->id].height;

    //[4]. Correct the Fin information according to the rotation and flip result.
    if(mod->rotate)
        swap(mod_mf.Dtop, mod_mf.Dbot);

    if(mod->flip)
        swap(mod_mf.Dtop, mod_mf.Dbot);

    Dbot = mod_mf.Dbot;
    Dfin = mod_mf.Dfin;

    //[5]. Placement of the root module.
    if(mod->parent == nullptr){
        mod_mf.x  = 0;
        mod_mf.y = calc_newY(Dbot,Dfin,0);

        // check the root module is a minimum separation module.
        for(int i = 0; i < min_seps.size(); i++)
        {
            if(mod->id == min_seps[i].mod)
            {
                mod_mf.x  += min_seps[i].dis;
                mod_mf.y = calc_newY(Dbot,Dfin,0);  // proceed fin-alignment constraint.
                mod_mf.rx = w + mod_mf.x;

                if(mod_mf.y < min_seps[i].dis)  
               {
                    mod_mf.y += (ceil((float)min_seps[i].dis / (float)Dfin) * Dfin - min_seps[i].dis); // proceed the minimum_separatoin constraint
                }
                mod_mf.ry = mod_mf.y + h;
            }
        }
        newY = mod_mf.y;
        correct_location_by_preplaced(mod_mf.x, mod_mf.y, mod); // proceed the preplaced constraint.
        // make fin alignment
        curY = mod_mf.y;
        // make fin alignment after modifying the position according to the preplace module
        if(curY != newY){
            newY = calc_newY(Dbot, Dfin, curY);
            mod_mf.y = newY;
        }
        mod_mf.rx = mod_mf.x + w;
        mod_mf.ry = mod_mf.y + h;

        //[3]. Update Tiles.
        // solid tile.
        tile_map[mod->id][0].set_Val(mod_mf.x, mod_mf.y, mod_mf.rx, mod_mf.ry, true);
        // upper placing tile.
        tile_map[mod->id][1].set_Val(0, mod_mf.ry, mod_mf.rx, estimate_height, false);
        // contour tile.
        tile_map[mod->id][2].set_Val(mod_mf.rx, 0, estimate_width, estimate_height, false);

        return;
    }

    pid = mod->parent->id;
    //[4]. Children Node.
    //  [4.1] Left node.
    if(mod->parent->left == mod)
    {
        if(!tile_map[pid][2].is_Nil())
        {
            mod_mf.x = tile_map[pid][2].cornerstich_x;
            mod_mf.rx = mod_mf.x + w;

            temp = tile_map[pid][2].lb;    // get the current tile's lb tile
            Dtop = temp == NIL ? 0 : modules[temp].Dtop; 
            curY = temp == NIL ? 0 : tile_map[temp][0].ry;
            newY = calc_newY(Dbot,Dfin,curY);

            // minimum seperation.
            for(int i = 0; i < min_seps.size(); i++)
            {
                if(pid == min_seps[i].mod || mod->id == min_seps[i].mod){
                    mod_mf.x += min_seps[i].dis;
                    mod_mf.rx += min_seps[i].dis;
                }
            }
            mod_mf.y = newY;
            correct_location_by_preplaced(mod_mf.x, mod_mf.y, mod); // proceed the preplaced constraint.
            // make fin alignment
            curY = mod_mf.y;
            // make fin alignment after modifying the position according to the preplace module
            if(curY != newY){
                newY = calc_newY(Dbot, Dfin, curY);// proceed the fin-alignment constraint.
                mod_mf.y = newY;
            }
            mod_mf.rx = mod_mf.x + w;
            mod_mf.ry = mod_mf.y + h;

            update_tiles(mod,pid);  // update the tile map.
            return;
        }

        temp = tile_map[pid][0].tr;
        pid = temp;
    }
    // right node
    else
    {
        temp = pid;
    }
    // right node processing.
    mod_mf.x = tile_map[temp][1].cornerstich_x;
    mod_mf.rx = mod_mf.x + w;

    curY = tile_map[temp][0].ry;   // get Parent module's ry.
    max_y = curY;
    if(mod_mf.rx > tile_map[temp][1].rx)    // module's right side is over the parent's right.
    {
        do
        {
            if(tile_map[temp][0].tr == NIL)  // get Parent tiles's tr tile.
                break;

            temp = tile_map[temp][0].tr;
            curY = tile_map[temp][0].ry;
            max_y = max(max_y, curY);       // get the maxium bottom line of the tr tile and self.
        }while(mod_mf.rx > tile_map[temp][1].rx); // if the module's right side is not over the tr tile's right, finish it.
    }

    newY = calc_newY(Dbot,Dfin,max_y);  // proceed the fin line alignment constraint.
    mod_mf.y = newY;

    correct_location_by_preplaced(mod_mf.x, mod_mf.y, mod); // proceed the preplaced constraint.
    // make fin alignment
    curY = mod_mf.y;
    // make fin alignment after modifying the position according to the preplace module
    if(curY != newY){
        newY = calc_newY(Dbot, Dfin, curY); // proceed the fin align constraint.
        mod_mf.y = newY;
    }
    
    mod_mf.rx = mod_mf.x + w;
    mod_mf.ry = mod_mf.y + h;
    

    // minimum seperation constraint..
    for(int i = 0; i < min_seps.size(); i++)
    {
        if(pid == min_seps[i].mod || mod->id == min_seps[i].mod){
            // make separation
            if((mod_mf.y - curY) < min_seps[i].dis){

                //recorrect the minimum_distance according to the fin alignment
                 mod_mf.y += ceil( ((float)min_seps[i].dis - (mod_mf.y - curY)) / (float)Dfin ) * (float)Dfin;
                 mod_mf.ry = mod_mf.y + h;
                 mod_mf.x += min_seps[i].dis;
                 mod_mf.rx += min_seps[i].dis;
            }
        }
    }
    update_tiles(mod,pid,temp);  // update the tile map.
}

// Proceed the preplaced constraint.
void CB_Tree::correct_location_by_preplaced(int &x, int &y, Node* mod)
{
    int ppm_x,ppm_rx,ppm_y,ppm_ry,rx,ry,w,h;
    w = modules[mod->id].width;
    h = modules[mod->id].height;
    rx = x + w;
    ry = y + h;
    for(int i = 0; i < preplaced.size(); i++)
    {
        ppm_x = preplaced[i].x;
        ppm_y = preplaced[i].y;
        ppm_rx = preplaced[i].rx;
        ppm_ry = preplaced[i].ry;

        if( ( ( ppm_x >= x && ppm_x < rx ) || ( ppm_rx > x && ppm_rx <= rx ) )
            && ( ( ppm_y >= y && ppm_y < ry ) || ( ppm_ry > y && ppm_ry <= ry ) ) )  // if the placed module is overlapped with the preplaced module.
        {
            y = ppm_ry;  // avoid the preplaced module.
        }
    }
}

// Proceed the fin-alignment constraint.
int  CB_Tree::calc_newY(int Dbot, int Dfin, int curY)
{
    if(Dfin == 0) // if the module doesn't have layout patterns and doesn't have the fin line.
    {
        return curY; // we don't consider the fin-alignment constraint.
    }

    return ceil( ((float)curY + (float)Dbot) / (float)Dfin ) * (float)Dfin - Dbot;
}

// Update the tile map.
void CB_Tree::update_tiles(Node *mod, int pid, int contour){
    int mid = mod->id;
    int p_bl_id = tile_map[pid][0].bl;  // get the bl tile of the parent module.
    int p_tr_id = tile_map[pid][0].tr;  // get the top tile of the parent module.

    Module_Info mod_mf = modules_info[mid];

    // if the module is overflow the CB-Tree region, we increase the CB-Tree region.
    estimate_width = mod_mf.rx > estimate_width ? mod_mf.rx + 10 : estimate_width;  
    estimate_height = mod_mf.ry > estimate_height ? mod_mf.ry + 10 : estimate_height;

    // update tile.
    tile_map[mid][0].set_Val(mod_mf.x, mod_mf.y, mod_mf.rx, mod_mf.ry, true);
    // upper placing tile.
    tile_map[mid][1].set_Val(mod_mf.x, mod_mf.ry, mod_mf.rx, estimate_height, false);
    // contour tile.

    if(contour == NIL)  // if the module is placed in bottom side,
    {
        tile_map[mid][2].set_Val(mod_mf.rx, 0, estimate_width, estimate_height, false); // set the lb tile.
        tile_map[pid][2].set_Nil(); // parent's lb setNil.
        if(mod->parent->id == pid)  // left node.
        {
            tile_map[pid][0].tr = mid;  // parent.tr = child
            tile_map[mid][0].bl = pid;  // child.bl = parent
            return;
        }
        else // right node
        {
            tile_map[mid][0].set_Con(p_bl_id,NIL,p_tr_id,NIL); // child.bl = parent.bl, child.tr = parent.tr
            if(p_bl_id != NIL)  
                tile_map[p_bl_id][0].tr = mid; // {child.bl}.tr = child 
            if(p_tr_id != NIL)
                tile_map[p_tr_id][0].bl = mid; // {child.tr}.bl = child
            tile_map[pid][0].set_Con(NIL,NIL,NIL,NIL); // remove the parent's relationship
        }
    }
    else // There is a contour in the bottom of the module.
    {
        tile_map[mid][2].set_Nil(); // There is no empty area in the module's right side, 

        if(tile_map[contour][1].rx <= mod_mf.rx){  // contour.right <= child.right
            tile_map[contour][1].set_Nil();         //contour.tr = nil
            tile_map[mid][0].set_Con(p_bl_id,NIL,p_tr_id,NIL);  // child.bl = parent.bl, child.tr = parent.tr
            if(p_bl_id != NIL)
                tile_map[p_bl_id][0].tr = mid;  //{child.bl}.tr = child
            if(p_tr_id != NIL)
                tile_map[p_tr_id][0].bl = mid;  //{child.tr}.bl = child
            tile_map[pid][0].set_Con(NIL,NIL,NIL,NIL);

            if(tile_map[contour][0].tr == NIL) // if the contour is in the most-right side
            {
                tile_map[mid][2].set_Val(mod_mf.rx, 0, estimate_width, estimate_height, false);  //child has the empty area in its right side.
                tile_map[contour][2].set_Nil(); // There is no empty area in the contour's right side 
            }
        }
        else if(tile_map[contour][1].rx > mod_mf.rx) // contour.right > child.right
        {
            tile_map[contour][1].cornerstich_x = mod_mf.rx;  
            if(pid == contour){ //right node, parent is just contour
                if(p_bl_id != NIL)
                    tile_map[p_bl_id][0].tr = mid;  // {parent.bl}.tr = child
                tile_map[mid][0].bl = p_bl_id;      // child.bl = parent.bl
                tile_map[mid][0].tr = pid;          // child.tr = parent
                tile_map[pid][0].bl = mid;          // parent.bl = child
            }
            else // not right node.
            {
                tile_map[mid][0].set_Con(p_bl_id,NIL,p_tr_id,NIL);  // chile.bl = parent.bl, child.tr = parent.tr
                if(p_bl_id != NIL)
                    tile_map[p_bl_id][0].tr = mid;                  // {child.bl}.tr = child
                if(p_tr_id != NIL)      
                    tile_map[p_tr_id][0].bl = mid;                  // {child.tr}.bl = child
                tile_map[pid][0].set_Con(NIL,NIL,NIL,NIL);          // remove the parent's relationship
            }
        }



    }
}

/* Place the layout patterns */
void CB_Tree::place_patterns(Node* mod)
{
    int Dt,Dl,Dr,Db,l,r,b,t;
    if(modules_info[mod->id].Dfin == 0) // If the module doesn't have a patterns.
        return;
    for(int i = 0; i < layout_patterns[mod->id].size(); i++)
    {
        //[1]. calculate the pattern's boundary distance
        Dt = modules[mod->id].height - layout_patterns[mod->id][i].layout.top;
        Dr = modules[mod->id].width - layout_patterns[mod->id][i].layout.right;
        Db = layout_patterns[mod->id][i].layout.bottom;
        Dl = layout_patterns[mod->id][i].layout.left;

        l = Dl;
        r = modules[mod->id].width - Dr;
        b = Db;
        t = modules[mod->id].height - Dt;

        if(mod->rotate) // rotate, swap the left side and right side, bottom side and top side.
        {
            l = Dr;
            r = modules[mod->id].width - Dl;
            b = Dt;
            t = modules[mod->id].height - Db;
        }

        if(mod->flip) // swap the top and bottom side
        {
            b = Dt;
            t = modules[mod->id].height - Db;
        }

        //[2]. calculate the position of the layout pattern.
        modules_info[mod->id].patterns[i].layout.left = l + modules_info[mod->id].x;
        modules_info[mod->id].patterns[i].layout.right = r + modules_info[mod->id].x;
        modules_info[mod->id].patterns[i].layout.bottom = b + modules_info[mod->id].y;
        modules_info[mod->id].patterns[i].layout.top = t + modules_info[mod->id].y;

        modules_info[mod->id].patterns[i].mask_id = layout_patterns[mod->id][i].mask_id;
        modules_info[mod->id].patterns[i].internal = layout_patterns[mod->id][i].internal;
    }

}

// Calculate the density area in the CB-Tree. [not used in WB-Tree]
double CB_Tree::getDensityArea()
{
    double area = 0;
    vector<Node*> nodes = allnodes();
    for(int i = 0; i < nodes.size(); i++)
    {
        if(modules[nodes[i]->id].Dfin == 0)
            continue;

        for(int j = 0; j < layout_patterns[nodes[i]->id].size(); j++)
        {
            // calculate the sum area of the layout patterns.
            area += (layout_patterns[nodes[i]->id][j].layout.right - layout_patterns[nodes[i]->id][j].layout.left)
                    * (layout_patterns[nodes[i]->id][j].layout.top - layout_patterns[nodes[i]->id][j].layout.bottom);
        }
    }
    return area*1e-6;
}

// Show the CB-Tree's structure into the console.
void CB_Tree::show_tree()
{
    auto nodes = allnodes();
    cout <<"root : "<<nodes[0]->id<<endl;
    for(int i=0; i < nodes.size(); i++){
        cout << nodes[i]->id << ": ";
        cout << (nodes[i]->left == nullptr ? -1 : nodes[i]->left->id) << " ";
        cout << (nodes[i]->parent == nullptr ? -1 : nodes[i]->parent->id) << " ";
        cout << (nodes[i]->right == nullptr ? -1 : nodes[i]->right->id) <<endl;
    }
}

//---------------------------------------------------------------------------
//   Simulated Annealing Permutation Operations [not used in WB-Tree]
//---------------------------------------------------------------------------

void CB_Tree::perturb(){
    auto nodes = allnodes();
    if(nodes.size()<4)
        return;

    int p,n;
    n = rand()%nodes.size();  //modules_N;

    if(rotate_rate > rand_01()){
        //    changed_nodes.push_back(nodes[n]);
        nodes[n]->rotate = !nodes[n]->rotate;
        if(rand_bool()) nodes[n]->flip = !nodes[n]->flip;
    }
    else{

        if(swap_rate >rand_01()){
            do{
                p = rand()%nodes.size(); //modules_N;
            }while(n==p||nodes[n]->parent==nodes[p]||nodes[p]->parent==nodes[n]);

            swap_node(nodes[p],nodes[n]);   // [TODO]. refer the swap in vector

        }else{
            do{
                p = rand()%nodes.size(); //modules_N;
            }while(n==p);

            delete_node(nodes[n]);           // [TODO]. refer the delete in vector
            insert_node(nodes[p],nodes[n]); // [TODO]. refer the insert in vector
        }
    }

}

void CB_Tree::swap_node(Node *n1, Node *n2){

    if(n1->left!=nullptr){
        //add_changed_nodes(n1.left);
        n1->left->parent  = n2;
    }
    if(n1->right!=nullptr){
        //add_changed_nodes(n1.right);
        n1->right->parent = n2;
    }
    if(n2->left!=nullptr){
        //add_changed_nodes(n2.left);
        n2->left->parent  = n1;
    }
    if(n2->right!=nullptr){
        //add_changed_nodes(n2.right);
        n2->right->parent = n1;
    }

    if(n1->parent != nullptr){
        //add_changed_nodes(n1.parent);
        if(n1->parent->left==n1)
            n1->parent->left  = n2;
        else
            n1->parent->right = n2;
    }else{
        nodes_root = n2;
    }

    if(n2->parent != nullptr){
        //add_changed_nodes(n2.parent);
        if(n2->parent->left==n2)
            n2->parent->left  = n1;
        else
            n2->parent->right = n1;
    }else{
        //    changed_root = n2.id;
        nodes_root = n1;
    }

    swap(n1->left,n2->left);
    swap(n1->right,n2->right);
    swap(n1->parent,n2->parent);
}
//----------------------------------------------------------------

// Find node with modeul id.
Node* CB_Tree::find_node_by_id(int moduleID)
{
    auto nodes = allnodes();
    for(int i=0; i<nodes.size(); i++)
        if(nodes[i]->id==moduleID)
            return nodes[i];
    return nullptr;
}

// Find node randomly.
Node* CB_Tree::find_node_random()
{
    auto nodes = allnodes();
    if(nodes.empty() || nodes.size() == 0)
        return nullptr;
    int i = rand() % nodes.size();
    return nodes[i];
}

// Insert the node with module id
void CB_Tree::insert_node_by_id(Node* parent, int moduleId)
{
    int j;
    // [0]. If there is no node,inserted node is root node.
    if(parent == nullptr){
        vector<int> inds;
        inds.push_back(moduleId);
        init_with_node_indices(inds); // set the nodes to the CB-Tree
        return;
    }
    // [1]. check if the module ID is exist already.
    auto nodes = allnodes();
    for(int i=0; i<nodes.size(); i++)
        if(nodes[i]->id==moduleId)
            return;
    // [2]. make new node.
    Node* node = (Node*) new unsigned char[sizeof(Node)];
    memset(node, 0, sizeof(Node));
    node->id = moduleId;
    // [3]. insert new node.
    insert_node(parent, node);  // insert the new node.
    nodes_N = allnodes().size();
}

// Swap node by id in CB-Tree [not used in WB-Tree]
void CB_Tree::swap_node_by_id (int m1, int m2)
{
    Node *n1 = find_node_by_id(m1);
    Node *n2 = find_node_by_id(m2);
    Node *t;
    if(n1 == nullptr || n2 == nullptr){
        cout<<"incorrect mod number."<<endl;
    }

    if(n1->parent == n2 || n2->parent == n1)
    {
        swap(n1,n2);

        if(n1->parent == nullptr){
            nodes_root = n1;
        }else if(n2->parent == nullptr){
            nodes_root = n2;
        }

    }
    else{
        swap_node(n1,n2);
    }

}

// insert Node.
void CB_Tree::insert_node(Node *parent, Node *node){
    node->parent = parent;
    bool edge = rand_bool();

    if(edge){ // insert as left
        //add_changed_nodes(parent.left);
        node->left  = parent->left;
        node->right = nullptr;
        if(parent->left!=nullptr)
            parent->left->parent = node;

        parent->left = node;

    }else{     // insert as right
        //add_changed_nodes(parent.right);
        node->left  = nullptr;
        node->right = parent->right;
        if(parent->right!=nullptr)
            parent->right->parent = node;

        parent->right = node;
    }
}

// Remove random node
int CB_Tree::take_node_random()
{
    auto nodes = allnodes();
    int i= rand()%nodes.size();
    int ModuleId = nodes[i]->id;
    auto node = nodes[i];
    delete_node(nodes[i]);
    SAFE_DELETE(node);
    nodes_N = allnodes().size();
    return ModuleId;  // return the deleted node
}

// remove the node with module id
bool CB_Tree::take_node(int mod_id)
{
    auto nodes = allnodes();
    for(int i = 0; i < nodes.size(); i++)
    {
        if(nodes[i]->id == mod_id){
            auto node = nodes[i];
            delete_node(nodes[i]);
            SAFE_DELETE(node);
            nodes_N = allnodes().size();
            return true; // succeed.
        }
    }

    return false; // there is no such a node.
}

// delete node.
void CB_Tree::delete_node(Node *node){
    Node* child    = nullptr;	// pull which child
    Node* subchild = nullptr;   // child's subtree
    Node* subparent= nullptr;

    if(!node->isleaf()){
        bool left= rand_bool();			// choose a child to pull up
        if(node->left ==nullptr) left=false;
        if(node->right==nullptr) left=true;

        //add_changed_nodes(node.left);
        //add_changed_nodes(node.right);

        if(left){
            child = node->left;			// child will never be NIL
            if(node->right!=nullptr)
            {
                subchild  = child->right;
                subparent = node->right;
                node->right->parent = child;
                child->right = node->right;	// abut with node's another child
            }
        }
        else{
            child = node->right;
            if(node->left!=nullptr)
            {
                subchild  = child->left;
                subparent = node->left;
                node->left->parent = child;
                child->left = node->left;
            }
        }
        //add_changed_nodes(subchild);
        child->parent = node->parent;
    }

    if(node->parent == nullptr){			// root
        //    changed_root = nodes_root;
        nodes_root = child;
    }else{					// let parent connect to child
        //add_changed_nodes(node.parent);
        if(node == node->parent->left)
            node->parent->left  = child;
        else
            node->parent->right = child;
    }

    // place subtree
    if(subchild != nullptr){
        Node *sc = subchild;
        assert(subparent != nullptr);

        while(1){
            Node *p = subparent;

            if(p->left==nullptr || p->right==nullptr){
                //add_changed_nodes(p.id);

                sc->parent = p;
                if(p->left==nullptr) p->left = sc;
                else p->right = sc;
                break;
            }else{
                subparent = (rand_bool() ? p->left : p->right);
            }
        }
    }
}

// copy the tree--- traverse algorithm, It continues to iterate looking for its children.
void _copyTree(Node* parent, Node* node, bool is_left, vector<Node> &tree)
{
    if(node==nullptr)
    {
        (is_left? parent->left : parent->right) = nullptr;
        return;
    }

    tree.push_back(*node);
    //cout<<node->id<<":"<<(node->left==nullptr ? -1 : node->left->id)<<" "<<(node->parent==nullptr ? -1 : node->parent->id)<<" "<<(node->right==nullptr ? -1 : node->right->id)<<endl;
    Node& currnode = tree.back();
    currnode.parent = parent;

    if(parent!=nullptr)
        (is_left? parent->left : parent->right) = &tree.back();

    _copyTree(&currnode, node->left, true, tree);
    _copyTree(&currnode, node->right, false, tree);

    //cout<<currnode.id<<":"<<(currnode.left==nullptr ? -1 : currnode.left->id)
    //        <<" "<<(currnode.parent==nullptr ? -1 : currnode.parent->id)<<" "<<(currnode.right==nullptr ? -1 : currnode.right->id)<<endl;
}

// Start of the copy_tree algorithm
void CB_Tree::copy_tree(vector<Node> &tree)
{
    tree.reserve(vector_max_size);
    tree.clear();
    _copyTree(nullptr, nodes_root, true, tree); // copytree algorithm starts from the root node.
}

// get the nodes array of the CB-Tree nodes. [not used in CB-Tree]. It also traverses looking for its children
void _traverse(Node* node, vector<Node*>& nodes)
{
    if(node==nullptr)
        return;

    nodes.push_back(node);
    _traverse(node->left, nodes);
    _traverse(node->right, nodes);
}

// Get All nodes of CB-Tree. the start point of the _traverse algorithm
vector<Node*> CB_Tree::allnodes()
{
    vector<Node*> r;
    _traverse(nodes_root, r); // start traverse.
    return r;
}

// Delete the CB-Tree
void CB_Tree::destroy(){
  auto nodes = allnodes();
   for(int i = 0; i < nodes.size(); i++)
   {
     SAFE_DELETE(nodes[i]);
   }
   nodes.clear();
}
