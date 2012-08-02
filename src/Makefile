marccd-objs =  MarccdInterface.o MarccdCamera.o MarccdReader.o
SRCS = $(marccd-objs:.o=.cpp) 

CXXFLAGS += -I../include -I../../../hardware/include -I../../../common/include \
			-I../../../third-party/Processlib/core/include \
			-I../../../../include/DiffractionImage \
			-I../../../third-party/yat/include \
			-Wall -pthread -fPIC -g

#CXXFLAGS += -fPIC -I../../../third-party/Processlib/core/include \
#			-I../../../../DiffractionImage \
#			-I../../../third-party/yat/include \
#			-I../include \
#			-I../../../hardware/include \
#			-I../../../common/include 
#			-pthread -fPIC -g \
#			-L../../../third-party/yat/src/.libs -lyat \
#			-L../../../../DiffractionImage/.libs -lDiffractionImage

all:	Marccd.o

Marccd.o:	$(marccd-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
