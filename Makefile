all: iface lcd ocxo_ctrl pfd

all: FORCE
	$(MAKE) -C iface
	$(MAKE) -C lcd
	$(MAKE) -C ocxo_ctrl
	$(MAKE) -C pfd

clean:
	$(MAKE) -C iface clean
	$(MAKE) -C lcd clean
	$(MAKE) -C ocxo_ctrl clean
	$(MAKE) -C pfd clean

FORCE: ;
