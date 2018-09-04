.PHONY: all init nanopb_common clean

NANOPB_DIR = nanopb

NANOPB_COMMON_FILES = pb.h pb_common.c pb_common.h pb_decode.c pb_decode.h pb_encode.c pb_encode.h

all: init sensorbug.pb.c sensorbug.pb.h $(NANOPB_COMMON_FILES)

# Mandatory first step to use Nanopb protoc plugin
init:
	make -C $(NANOPB_DIR)/generator/proto

# Copies in Nanopb common files
%.c: $(NANOPB_DIR)/%.c
	cp $^ ./
%.h: $(NANOPB_DIR)/%.h
	cp $^ ./

# Generates Nanopb stub libs from .proto file
%.pb.c %.pb.h: %.proto
	protoc --plugin=protoc-gen-nanopb=$(NANOPB_DIR)/generator/protoc-gen-nanopb --nanopb_out=. $<

clean:
	# No clean target in Nanopb plugin
	$(RM) *.pb.[ch]
	$(RM) $(NANOPB_COMMON_FILES)
