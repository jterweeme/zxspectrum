all: spectrum.sof

spectrum.sof:
	quartus_sh --flow compile spectrum -c spectrum

clean:
	rm -Rvf db incremental_db simulation greybox_tmp
	rm -Rvf *.sof *.pof *.bak *.rpt *.jdi *.summary *.pin *.done *.qws *.smsg
	rm -Rvf PLLJ_PLLSPE_INFO.txt


