#Project: B*-tree based floorplanning/placement
#Advisor: Yao-Wen Chang  <ywchang@cis.nctu.edu.tw>
#Author:  Jer-Ming Hsu	 <barz@cis.nctu.edu.tw>
#	  Hsun-Cheng Lee <gis88526@cis.nctu.edu.tw>
#Sponsors: NSC, Taiwan; Arcadia, Inc; UMC Corp.
#Version: 1.0
#Date:    7/19/2000

.SUFFIXES: .cc .o
SHELL=/bin/sh
CXX=g++
DEBUG= -g
OPT= -O2 -DNDEBUG
CXXFLAGS= -c $(DEBUG) $(OPT)
LDFLAGS= 

###########################################################################

LIBS = -lstdc++
OBJS = fplan.o sa.o
B_OBJS  = cbtree.o wbtree.o wbtree_main.o $(OBJS)
SRCS = ${OBJS:%.o=%.cc}

all:    wbtree 

wbtree: $(B_OBJS)
	$(CXX) -o wbtree $(B_OBJS) $(LIBS) $(LDFLAGS)

%.o : %.cc %.h fplan.h cbtree.h
	$(CXX) $*.cc $(CXXFLAGS)

%.o : %.cc  fplan.h cbtree.h
	$(CXX) $*.cc $(CXXFLAGS)

clean: 
	rm -f *.o wbtree *~

compact : wbtree
	strip $?

