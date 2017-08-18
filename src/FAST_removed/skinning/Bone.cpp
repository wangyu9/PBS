#ifndef IGL_HEADER_ONLY
#define IGL_HEADER_ONLY
#endif

#include <algorithm>
#include <cstdio>
#include <igl/verbose.h>
#include <igl/PI.h>
#include <igl/project.h>
#include <igl/unproject.h>
#include "Bone.h"
#include "draw_point.h"
#include "draw_directed_line_segment.h"
#include "inside_point.h"
#include "inside_line_segment.h"
#include "view_axis.h"
#include "up_axis.h"
#include "right_axis.h"

#if __APPLE__
#  include <OpenGL/gl.h>
#else
#  ifdef _WIN32
#    include <Windows.h>
#  endif
#  include <GL/gl.h>
#endif

using namespace std;
using namespace igl;

static double POINT_COLOR[3] = {239./255.,213./255.,46./255.};
static double FREE_POINT_COLOR[3] = {113./255.,239./255.,46./255.};
static double FIXED_POINT_COLOR[3] = {239./255.,113./255.,46./255.};
//static double TAIL_POINT_COLOR[3] = {200./255.,200./255.,200./255.};
static double HOVER_COLOR_OFFSET[3] = {-0.1,-0.1,-0.1};
static double DIRECTED_LINE_SEGMENT_COLOR[3] = {106./255.,106./255.,255./255.};
static double SELECTED_DIRECTED_LINE_SEGMENT_COLOR[3] = {
  DIRECTED_LINE_SEGMENT_COLOR[0] - 0.5,
  DIRECTED_LINE_SEGMENT_COLOR[1] - 0.5,
  DIRECTED_LINE_SEGMENT_COLOR[2] - 0.5};

static const double BONE_WI_UNSET = -3;
static const double BONE_POINT_RADIUS = 7;
static const double BONE_DIRECTED_LINE_SEGMENT_WIDTH = 8;

Bone::Bone(
  Bone * parent_,
  const Vec3 & offset_):
    //public
    offset(offset_),
    rotation(Quat(1,0,0,0)),
    translation(Vec3(0,0,0)),
    stretch(1.0),
    twist(0),
    is_line_segment_selected(false),
    is_line_segment_hover(false),
    is_tip_selected(false),
    is_tip_hover(false),
    tip_dof_type(NORMAL_DOF),
    last_T(Tform3::Identity()),
    view_vector(VIEW_VECTOR),
    // private
    parent(NULL),
    wi(BONE_WI_UNSET),
    draw_according_to_last_T(false),
    draw_connected_to_parent(false),
    average_children_tails_to_draw_non_weighted_roots(false)
{
  if(parent_!=NULL)
  {
    set_parent(parent_);
  }
  //verbose("New bone\n");
}

Bone::Bone(const Bone * that)
{
  // Copy everything that is safe to copy
  offset = that->offset;
  rotation = that->rotation;
  translation = that->translation;
  stretch = that->stretch;
  twist = that->twist;
  is_line_segment_selected = that->is_line_segment_selected;
  is_line_segment_hover = that->is_line_segment_hover;
  is_tip_selected = that->is_tip_selected;
  is_tip_hover = that->is_tip_hover;
  tip_dof_type = that->tip_dof_type;
  last_T = that->last_T;
  wi = that->wi;
  draw_according_to_last_T = that->draw_according_to_last_T;
  draw_connected_to_parent = that->draw_connected_to_parent;
  average_children_tails_to_draw_non_weighted_roots = 
    that->average_children_tails_to_draw_non_weighted_roots;
}

Bone::~Bone()
{
  vector<Bone*>::iterator cit = children.begin();
  while(cit != children.end())
  {
    // clear the memory of this child
    delete (* cit);
    // erase pointer from list, returns next element in iterator
    cit = children.erase(cit);
  }
  //verbose("Bone destroyed\n");
}

Bone * Bone::set_wi(int wi)
{
  assert(wi != BONE_WI_UNSET);
  this->wi = wi;
  return this;
}

int Bone::get_wi() const
{
  return wi;
}

bool Bone::wi_is_set() const
{
  return wi != BONE_WI_UNSET;
}

Bone * Bone::set_parent(Bone * parent)
{
  // Don't make roots this way
  assert(parent != NULL);
  // Only let this happen once!
  assert(this->parent == NULL);
  this->parent = parent;
  // Tell parent she has just given birth
  this->parent->children.push_back(this);

  return this;
}

const Bone * Bone::get_parent() const
{
  return this->parent;
}

std::vector<Bone*> Bone::get_children() const
{
  return children;
}

bool Bone::is_root() const
{
  return parent==NULL;
}

void Bone::reset()
{
  translation = Vec3(0,0,0);
  rotation = Quat(1,0,0,0);
  stretch = 1.0;
  twist = 0.0;
  last_T = Tform3::Identity();
}

Bone * Bone::set_draw_according_to_last_T(const bool v)
{
  draw_according_to_last_T = v;
  return this;
}

Bone * Bone::set_draw_connected_to_parent(const bool v)
{
  draw_connected_to_parent = v;
  return this;
}

Bone * Bone::set_average_children_tails_to_draw_non_weighted_roots(const bool v)
{
  average_children_tails_to_draw_non_weighted_roots = v;
  return this;
}

Bone * Bone::set_view_vector(const ViewVector v)
{
  view_vector = v;
  return this;
}

bool Bone::down(
  int x,
  int y, 
  bool right_click, 
  bool shift_down, 
  bool control_down, 
  bool meta_down)
{
  // Keep track of click position
  down_x = x;
  down_y = y;
  // reset to false on each mouse down
  is_tip_hover = false;
  is_line_segment_hover = false;

  // Check endpoint
  Vec3 d = tip(draw_according_to_last_T,
    average_children_tails_to_draw_non_weighted_roots && is_root()&&wi<0);
  bool in_tip = inside_point(x,y,d[0],d[1],d[2],BONE_POINT_RADIUS);

  // Check bone segment
  bool in_line_seg = false;
  // Don't check roots and don't check if we just found out that we're in the
  // tip
  if(!is_root() && !in_tip)
  {
    Vec3 s;
    if(draw_connected_to_parent)
    {
      s = parent->tip(
        draw_according_to_last_T,
        parent->average_children_tails_to_draw_non_weighted_roots&&
          parent->is_root()&&parent->wi<0);
    }else
    {
      s = tail(draw_according_to_last_T);
    }

    if(!inside_point(x,y,s[0],s[1],s[2],BONE_POINT_RADIUS))
    {
      in_line_seg = inside_line_segment(
        x,y,
        s[0],s[1],s[2],
        d[0],d[1],d[2],
        BONE_DIRECTED_LINE_SEGMENT_WIDTH);
    }
  }

  // Keep track of what used to be selected
  was_tip_selected = is_tip_selected;
  was_line_segment_selected = is_line_segment_selected;

  is_tip_selected= 
    (in_tip ? 
       (control_down ? !is_tip_selected : true) : 
       control_down&&is_tip_selected);
  is_line_segment_selected = 
    (in_line_seg ? 
      (control_down ? !is_line_segment_selected : true) : 
      control_down&&is_line_segment_selected);

  is_selected = is_tip_selected || is_line_segment_selected;
  is_down = in_tip || in_line_seg;
  last_x = x;
  last_y = y;
  return is_down;
}

bool Bone::up(
  int x,
  int y, 
  bool right_click, 
  bool shift_down, 
  bool control_down, 
  bool meta_down)
{
  is_down = false;
  return false;
}

bool Bone::drag(
  int x,
  int y, 
  bool right_click, 
  bool shift_down, 
  bool control_down, 
  bool meta_down)
{
  if(is_selected)
  {
    if(right_click)
    {
      if(is_tip_selected)
      {
        Vec3 axis;
        view_axis(&axis[0],&axis[1],&axis[2]);
        switch(view_vector)
        {
          case VIEW_VECTOR:
            view_axis(&axis[0],&axis[1],&axis[2]);
            break;
          case UP_VECTOR:
            up_axis(&axis[0],&axis[1],&axis[2]);
            break;
          case RIGHT_VECTOR:
            right_axis(&axis[0],&axis[1],&axis[2]);
            break;
        }
        // Normalize the axis
        axis.normalize();
        double angle = -2.0*PI*360.0*( (x-last_x)/ 400.0)/360.0;
        // Offset *ALL* rotations so that rotations are about the parent (self in
        // case of roots)
        Vec3 tail = offset;
        if(!is_root())
        {
          tail = parent->rest_tip();
        }
        // old translation
        Vec3 ot = (rotation*(translation + tail))-tail;
        // update rotation
        //rotation = rotation * Quat(Eigen::AngleAxis<double>(angle,axis));
        rotation = Quat(Eigen::AngleAxis<double>(angle,axis))*rotation;
        // center rotations around offset point for roots
        // update translation
        translation = ((rotation.inverse() * (tail+ot)) - (tail));
      }else if(is_line_segment_selected)
      {
        // twist about bone
        assert(!is_root());
        Vec3 d = rest_tip();
        Vec3 s = parent->rest_tip();
        // bone axis
        Vec3 axis = s-d;
        // Normalize the axis
        axis.normalize();
        double angle = -2.0*PI*360.0*( (x-last_x)/ 400.0)/360.0;
        // Offset *ALL* rotations so that rotations are about the parent (self in
        // case of roots)
        Vec3 tail = offset;
        if(!is_root())
        {
          tail = parent->rest_tip();
        }
        // old translation
        Vec3 ot = (rotation*(translation + tail))-tail;
        // update rotation
        rotation = rotation * Quat(Eigen::AngleAxis<double>(angle,axis));
        // center rotations around offset point for roots
        // update translation
        translation = ((rotation.inverse() * (tail+ot)) - (tail));

      }else
      {
        assert(false);
      }
    }else
    {
      // Get position of tip
      Vec3 t = tip(draw_according_to_last_T,
        average_children_tails_to_draw_non_weighted_roots&&is_root()&&wi<0);
      Vec3 tc = t;
      // Get screen position of tip
      Vec3 s_t;
      project(t[0],t[1],t[2],&s_t[0],&s_t[1],&s_t[2]);
      // Move tip in screen space according to mouse move since last
      s_t[0] += (x-last_x);
      s_t[1] += (y-last_y);
      // unproject back to 3d
      unproject(s_t[0],s_t[1],s_t[2],&t[0],&t[1],&t[2]);
      // affective translation, this is the translation we want to show up in
      // affine()
      Vec3 at = (t-tc);

      Vec3 p_at(0,0,0);
      if(parent != NULL)
      {
        p_at = parent->affine().translation();
      }
      // set translation to put rest tip at new tip position
      //translation += (rotation.inverse() * orientation()).inverse() * at;
      translation += orientation().inverse() * at;
      // New tip
      Vec3 nt = tip(draw_according_to_last_T,
        average_children_tails_to_draw_non_weighted_roots&&is_root()&&wi<0);
    }
  }
  // keep track of last mouse position
  last_x = x;
  last_y = y;
  return is_selected;
}

bool Bone::move(
  int x,
  int y, 
  bool /*shift_down*/, 
  bool /*control_down*/, 
  bool /*meta_down*/)
{
  assert(!is_down);
  // reset to false on each mouse move
  is_tip_hover = false;
  is_line_segment_hover = false;

  // Check endpoint
  Vec3 d = tip(draw_according_to_last_T,
    average_children_tails_to_draw_non_weighted_roots&&is_root()&&wi<0);
  bool in_tip = inside_point(x,y,d[0],d[1],d[2],BONE_POINT_RADIUS);
#ifdef EXTREME_VERBOSE
  verbose("(%d %d) in (%g %g %g, %g): %s\n",
    x,y,
    d[0],d[1],d[2],
    BONE_POINT_RADIUS,(in_tip?"true":"false")
    );
#endif
  is_tip_hover = in_tip;

  // Check bone segment
  bool in_line_seg = false;
  if(!is_root() && !in_tip)
  {
    Vec3 s;
    if(draw_connected_to_parent)
    {
      s = parent->tip(
        draw_according_to_last_T,
        parent->average_children_tails_to_draw_non_weighted_roots&&
          parent->is_root()&&parent->wi<0);
    }else
    {
      s = tail(draw_according_to_last_T);
    }

    if(!inside_point(x,y,s[0],s[1],s[2],BONE_POINT_RADIUS))
    {
      in_line_seg = inside_line_segment(
        x,y,
        s[0],s[1],s[2],
        d[0],d[1],d[2],
        BONE_DIRECTED_LINE_SEGMENT_WIDTH);
    }
  }
  is_line_segment_hover = in_line_seg;

  is_hover = is_tip_hover || is_line_segment_hover;

  return false;
}

bool Bone::inside(
  int x,
  int y)
{
  // Check tip
  // get tip
  Vec3 d = tip(draw_according_to_last_T,
    average_children_tails_to_draw_non_weighted_roots&&is_root()&&wi<0);
  bool in_tip = inside_point(x,y,d[0],d[1],d[2],BONE_POINT_RADIUS);
  // Check bone segment
  bool in_line_seg = false;
  if(!is_root() && !in_tip)
  {
    Vec3 s;
    if(draw_connected_to_parent)
    {
      s = parent->tip(
        draw_according_to_last_T,
        parent->average_children_tails_to_draw_non_weighted_roots&&
          parent->is_root()&&parent->wi<0);
    }else
    {
      s = tail(draw_according_to_last_T);
    }

    if(!inside_point(x,y,s[0],s[1],s[2],BONE_POINT_RADIUS))
    {
      in_line_seg = inside_line_segment(
        x,y,
        s[0],s[1],s[2],
        d[0],d[1],d[2],
        BONE_DIRECTED_LINE_SEGMENT_WIDTH);
    }
  }
  return in_tip || in_line_seg;
}

void Bone::draw()
{
  if(parent != NULL)
  {
    Vec3 d = tip(draw_according_to_last_T,
      average_children_tails_to_draw_non_weighted_roots&&is_root()&&wi<0);
    Vec3 s;
    if(draw_connected_to_parent)
    {
      s = parent->tip(
        draw_according_to_last_T,
        parent->average_children_tails_to_draw_non_weighted_roots&&
          parent->is_root()&&parent->wi<0);
    }else
    {
      s = tail(draw_according_to_last_T);
    }

    double lcolor[3];
    line_segment_color(lcolor);

    // Draw directed line segment "bone"
    glColor3d(lcolor[0],lcolor[1],lcolor[2]);
    draw_directed_line_segment(
      s[0],s[1],s[2],
      d[0],d[1],d[2],
      BONE_DIRECTED_LINE_SEGMENT_WIDTH);
    // Draw tail (might not be at same position as parent's tip)
    double pcolor[3];
    this->parent->tip_color(pcolor);
    // Increase brightness and average with gray
    pcolor[0] = 0.8*(1.0-0.2*(1.0-pcolor[0]))+0.2*0.5;
    pcolor[1] = 0.8*(1.0-0.2*(1.0-pcolor[1]))+0.2*0.5;
    pcolor[2] = 0.8*(1.0-0.2*(1.0-pcolor[2]))+0.2*0.5;
    glColor3d(pcolor[0],pcolor[1],pcolor[2]);
    draw_point(s[0],s[1],s[2],BONE_POINT_RADIUS,false);
    // Draw parent's tip again to make sure it shows up on top
    this->parent->draw_tip();
  }

  this->draw_tip();
}

void Bone::draw_tip()
{
  // only draw if this tip is not a non-weighted root or we're drawing
  // non-weighted roots as the average of children's tails
  if(average_children_tails_to_draw_non_weighted_roots||!(is_root()&&wi<0))
  {
    Vec3 d = tip(draw_according_to_last_T,
      average_children_tails_to_draw_non_weighted_roots&&is_root()&&wi<0);
    // get foreground color for points
    double pcolor[3];
    tip_color(pcolor);
    glColor3d(pcolor[0],pcolor[1],pcolor[2]);
    draw_point(d[0],d[1],d[2],BONE_POINT_RADIUS,is_tip_selected);
  }
}

void Bone::tip_color(double pcolor[3])
{
  switch(tip_dof_type)
  {
    case FIXED_DOF:
      copy(FIXED_POINT_COLOR,FIXED_POINT_COLOR+3,pcolor);
      break;
    case FREE_DOF:
      copy(FREE_POINT_COLOR,FREE_POINT_COLOR+3,pcolor);
      break;
    case NORMAL_DOF:
    default:
      copy(POINT_COLOR,POINT_COLOR+3,pcolor);
  }
  if(is_tip_hover && !is_down)
  {
    pcolor[0]+=HOVER_COLOR_OFFSET[0];
    pcolor[1]+=HOVER_COLOR_OFFSET[1];
    pcolor[2]+=HOVER_COLOR_OFFSET[2];
  }else if(is_tip_selected)
  {
    //copy(SELECTED_POINT_COLOR,SELECTED_POINT_COLOR+3,pcolor);
  }
}

void Bone::line_segment_color(double lcolor[3])
{
  // Get line segment color
  copy(DIRECTED_LINE_SEGMENT_COLOR,DIRECTED_LINE_SEGMENT_COLOR+3,lcolor);
  if(is_line_segment_hover && !is_down)
  {
    lcolor[0]+=HOVER_COLOR_OFFSET[0];
    lcolor[1]+=HOVER_COLOR_OFFSET[1];
    lcolor[2]+=HOVER_COLOR_OFFSET[2];
  }else if(is_line_segment_selected)
  {
    copy(
      SELECTED_DIRECTED_LINE_SEGMENT_COLOR,
      SELECTED_DIRECTED_LINE_SEGMENT_COLOR+3,
      lcolor);
  }
}

Quat Bone::orientation() const
{
  assert(stretch == 1);
  assert(twist == 0);
  Quat p_orientation;
  if(parent == NULL)
  {
    // Root base case
    // Identity rotation
    p_orientation = Quat(1,0,0,0);
  }else
  {
    p_orientation = parent->orientation();
  }
  // Compose with this rotation
  return p_orientation * rotation;
}

Vec3 Bone::tip(
  const bool according_to_last_T,
  const bool average_children_tails) const
{
  assert(stretch == 1);
  assert(twist == 0);
  if(average_children_tails)
  {
    Vec3 t(0,0,0);
    vector<Bone*> children = get_children();
    for(
      vector<Bone*>::iterator cit = children.begin(); 
      cit != children.end(); 
      cit++)
    {
      assert((*cit)->parent == this);
      t += (*cit)->tail(according_to_last_T);
    }
    t /= children.size();
    return t;
  }else
  {
    if(according_to_last_T)
    {
      return last_T * rest_tip();
    }else
    {
      return affine() * rest_tip();
    }
  }
}

Vec3 Bone::tail(const bool according_to_last_T) const
{
  assert(stretch == 1);
  assert(twist == 0);
  assert(parent!=NULL);
  if(according_to_last_T)
  {
    return last_T * parent->rest_tip();
  }else
  {
    return affine() * parent->rest_tip();
  }
}

Vec3 Bone::rest_tip() const
{
  Vec3 p_rest_tip;
  if(parent == NULL)
  {
    // Root base case
    p_rest_tip = Vec3(0,0,0);
  }else
  {
    p_rest_tip = parent->rest_tip();
  }
  return p_rest_tip + offset;
}

Vec3 Bone::rest_tail() const
{
  assert(parent!=NULL);
  return parent->rest_tip();
}

Tform3 Bone::affine() const
{
  assert(stretch == 1);
  assert(twist == 0);

  Tform3 p_affine;
  if(parent == NULL)
  {
    p_affine.setIdentity();
  }else
  {
    p_affine = parent->affine();
  }
  return p_affine.rotate(rotation).translate(translation);
}

Bone * Bone::set_is_tip_selected(const bool v)
{
  is_tip_selected = v;
  is_selected = is_tip_selected || is_line_segment_selected;
  return this;
}

Bone * Bone::set_is_line_segment_selected(const bool v)
{
  // roots don't have line segments
  assert(!v||!is_root());
  is_line_segment_selected = v;
  is_selected = is_tip_selected || is_line_segment_selected;
  return this;
}

void Bone::print() const
{
  printf("offset: %g %g %g\n",offset[0],offset[1],offset[2]);
  printf("translation: %g %g %g\n",translation[0],translation[1],translation[2]);
  printf("rotation: %g %g %g %g\n",rotation.w(),rotation.x(),rotation.y(),rotation.z());
  printf("stretch: %g\n",stretch);
  printf("twist: %g\n",twist);
  printf("\n");
}
