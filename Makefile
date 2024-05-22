#--------------------------------------------------
# Target file to be compiled by default
#--------------------------------------------------
MAIN = main
#--------------------------------------------------
# CC is the compiler to be used
#--------------------------------------------------
CC = gcc
#--------------------------------------------------
# CFLAGs are the options passed to the compiler
#--------------------------------------------------
CFLAGS = -Wall -lpthread -lrt -lm
#--------------------------------------------------
# OBJS are the objects files to be linked 
#--------------------------------------------------
OBJ1 = ptask
OBJ2 = graphic
OBJ3 = simulator
OBJ4 = user_input
OBJ5 = q_learning
OBJS = $(MAIN).o $(OBJ1).o $(OBJ2).o $(OBJ3).o $(OBJ4).o $(OBJ5).o
#---------------------------------------------------
# LIBS are the external libraries to be used
#---------------------------------------------------
LIBS= `allegro-config --libs`
#--------------------------------------------------
# Dependencies
#--------------------------------------------------
$(MAIN): $(OBJS)
	$(CC) -o $(MAIN) $(OBJS) $(LIBS) $(CFLAGS)

$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c

$(OBJ1).o: $(OBJ1).c
	$(CC) -c $(OBJ1).c

$(OBJ2).o: $(OBJ2).c
	$(CC) -c $(OBJ2).c

$(OBJ3).o: $(OBJ3).c
	$(CC) -c $(OBJ3).c

$(OBJ4).o: $(OBJ4).c
	$(CC) -c $(OBJ4).c

$(OBJ5).o: $(OBJ5).c
	$(CC) -c $(OBJ5).c

#---------------------------------------------------
# Command to delete object and executable files
# It can be specified inline: make clean
#---------------------------------------------------
clean:
	rm -rf *.o $(MAIN)
