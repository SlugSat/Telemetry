TESTO = test.o
TESTC = test.c
PROTO = Telemetry_Packet_Protocol.o
PROTOC = Telemetry_Packet_Protocol.c
PROTOH = Telemetry_Packet_Protocol.h
CONV = DateConversion.o
CONVC = DateConversion.c
OBJS = $(PROTO) $(TESTO) $(CONV)
LINKS = -g -lm -lpthread

all: test

test: $(OBJS) 
	gcc -o test $(OBJS) $(LINKS)
	
$(PROTO): $(PROTOC) $(PROTOH)
	gcc -c $(PROTOC) $(LINKS) -o $@

$(CONV): $(CONVC)
	gcc -c $(CONVC) $(LINKS) -o $@
	
$(TESTO): $(TESTC)
	gcc -c $(TESTC) $(LINKS) -o $@

clean:
	rm -rf  *.o test
	
make run: all 
	./test
