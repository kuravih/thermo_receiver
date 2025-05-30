CC = gcc
# =====================================================================================================================





# ==== receiver source files ==========================================================================================
RECEIVER_EXE = receiver
RECEIVER_SOURCES = receiver.c
RECEIVER_HEADERS = serial_link.h
RECEIVER_OBJS = $(addprefix ., $(addsuffix .o, $(basename $(notdir $(RECEIVER_SOURCES)))))
# ==== receiver source files ==========================================================================================





# ==== gcc flags ======================================================================================================
CFLAGS = -Wall -Wformat -pthread -O3
LIBS = -lpthread
CFLAGS_STD = $(CFLAGS) -std=c17
# ==== gcc flags ======================================================================================================





# ==== build rules ====================================================================================================
.%.o:%.c $(RECEIVER_HEADERS)
	$(CC) $(CFLAGS_STD) -c -o $@ $<

all: $(RECEIVER_EXE)
	@echo Build complete

$(RECEIVER_EXE): $(RECEIVER_OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean_receiver:
	rm -f $(RECEIVER_EXE) $(RECEIVER_OBJS)

clean: clean_receiver
# ==== build rules ====================================================================================================
