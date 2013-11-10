/* compile with:
 * gcc $(pkg-config cairo-xlib-xrender --cflags --libs) cairo-example.c
 */

// Making Cairo Work:
// 1. Download macports: http://www.macports.org/install.php
// 2. In ~, sudo port install cairo
// 3. Make sure the you have this in your path: PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

//INPUT:
// Figure 1 shows a GUI rendering of each stage of the path planning:
// Original Obstacles (plus wall boundary) & Start and End Points
// Grown Obstacles
// Visibility Graph
// Shortest Path


#include <cairo-xlib.h>
#include <X11/Xlib.h>
#include <stdio.h>
#include <iostream>

static cairo_surface_t *surface;
static cairo_t *cairo;
static Display *dpy;

void paint(Window w) {
  //printf("paint\n");
  cairo_set_line_width(cairo, 1);
  cairo_set_source_rgb(cairo, 255, 0, 0);
  //cairo_rectangle(cairo, 20, 20, 50, 50);
  //cairo_line_to (cairo, 1, 0.5);
  //cairo_stroke(cairo);

  cairo_set_source_rgb(cairo, 255, 0, 0);
  //cairo_rectangle(cairo, 100, 100, 50, 50);
  //cairo_stroke(cairo);

  //cairo_set_source_rgb(cairo, 255, 0, 0);
  //cairo_stroke(cairo);
  cairo_move_to(cairo, 25, 25);
  cairo_line_to (cairo, 400, 25);
  cairo_line_to(cairo, 400, 400);
  cairo_line_to(cairo, 25, 400);
  cairo_line_to(cairo, 25, 25);
  cairo_stroke(cairo);

}

int main() {
  dpy = XOpenDisplay(NULL);
  if (dpy == NULL) {
    fprintf(stderr, "Error: Can't open display. Is DISPLAY set?\n");
    return 1;
  }

  /*Creates display window - with size 500x500; background = white */
  Window w;
  w = XCreateSimpleWindow(dpy, RootWindow(dpy, 0),
                          0, 0, 500, 500, 0, 0, WhitePixel(dpy, 0));
  
  XSelectInput(dpy, w, StructureNotifyMask | ExposureMask);
  XMapWindow(dpy, w);

  surface = cairo_xlib_surface_create(dpy, w, DefaultVisual(dpy, 0), 500, 500);
  cairo = cairo_create(surface);

  while (1) {
    XEvent e;
    XNextEvent(dpy, &e);
    //printf("Got event: %d\n", e.type);

    switch (e.type) {
      case MapNotify:
      case Expose:
      case ConfigureNotify:
        paint(w);
        break;
    }
  }

  return 0;
}