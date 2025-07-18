#!/usr/bin/perl
#
# Copyright 2016-2025, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#
use READELF;
# read patch.elf and generate linker directive file

# call with "perl wiced-gen-ld.pl <args> patch.elf out=<*.ld>"
# the "patch.elf" is parsed to determine where to start the application memory
# the "out=<*.ld>" file is the output linker script
# arguments:
#   SRAM_BEGIN_ADDR=0x123456, SRAM_LENGTH=0x1234: start and length of SRAM section for app code and data
#   NUM_PATCH_ENTRIES=256: used to calculate reserved space in SRAM for patch entries
#   ISTATIC_BEGIN=0x123456, ISTATIC_LEN=0x1234: start and length of static section data, typically encrypted keys
#   PRAM_OBJ=abc.o;def.o;ghi.o: code/rodata for patch ram
#   XIP_LEN=0x1234, XIP_OBJ=abc.o;def.o;ghi.o: execute in place area for on-chip-flash starting at 0x504000, contains code and rodata from listed object files
#   UNUSED_BEGIN=0x123456, UNUSED_LEN=0x1234: start and length of an UNUSED section defined to avoid loading anything into
#   DIRECT_LOAD=1: indicates RAM download rather than FLASH
#   overlay=overlays.ld: a linker script snippet generated for an overlay section

# ram is extended with PRAM (taking from patch space) or XIP (taking from on-chip-flash, code or rodata only)
# .dsp_pram_section is for dsp download (libraries/codec_ak4679_lib/akm4679_dsp_*.h)

# look up for memory map and section input/output information
my $mem_lut = {
	".app_xip_area" => {
		"sections" => [
#    argument XIP_OBJ will add section matches $(XIP_OBJ)(.text.* .gnu.linkonce.t.* .rodata .constdata* .rodata.* .gnu.linkonce.r.* )
#    special section for Position Independent trampoline functions.
            "*(.pisec_*)",
			"KEEP(*(.data_in_xip_area))",
			"KEEP(*(.dsp_pram_section))",],
		"mem_type" => "xip_section",
		"pre" => ["xip_area_begin"],
		"post" => ["xip_area_end"],},
	".setup" => {
		"sections" => [
			"*(.init_code)",
			"*(.init_text)",
			"*(.spar_setup)"],
		"mem_type" => "ram",
		"pre" => [],
		"post" => [],},
	".text" => {
        "align" => 0,
		"sections" => [
			"*(.app_init_code)",
			"*(.emb_text)",
			"*(.text)",
			"*(.text_in_ram)",
			"*(.text.*)",
            "*(EmHeAacDec)",
			"*(.gnu.linkonce.t.*)",
			"*(.glue_7t)",
			"*(.glue_7)",
			"*(.vfp11_veneer)",
			"*(.v4_bx)",
			"*(.ARM.extab*)",
			"*(.gnu.linkonce.armextab.*)",
			"*(.ARM.exidx*)",
			"*(.gnu.linkonce.armexidx.*)",
			"*(.vpc_decoder_section)"],
		"mem_type" => "ram",
		"pre" => [],
		"post" => [],},
	".rodata" => {
        "align" => 32,
		"sections" => [
			"*(const_drom_var)",
			"*(.rodata)",
            "*(.constdata*)",
			"*(.rodata.*)",
			"*(.gnu.linkonce.r.*)",
			"*(.data.rom)"],
		"mem_type" => "ram",
		"pre" => [],
		"post" => [],},
	".data" => {
		"sections" => [
			"*(.data)",
			"*(.data.*)",
			"*(.gnu.linkonce.d.*)",
			"*(.init_patch_table)"],
		"mem_type" => "ram",
		"pre" => [],
		"post" => [],},
	".bss" => {
        "align" => 32,
		"sections" => [
			"*(.bss)",
			"*(.bss.*)",
			"*(.gnu.linkonce.b.*)",
			"*(COMMON)"],
		"mem_type" => "ram",
		"pre" => ["app_iram_bss_begin"],
		"post" => ["app_iram_bss_end"],},
	".unused" => {
		"sections" => [
			"*(.code_in_retention_ram)",
			"*(.data_in_retention_ram)"],
		"mem_type" => "unused",
		"pre" => ["unused_iram_begin"],
		"post" => ["unused_iram_end"],},
	".aon" => {
		"sections" => [
			"*(always_on_ram_var)",
			"*(.aon_code)",
			"*(.data_in_retention_ram)",
			"*(.code_in_retention_ram)"],
		"mem_type" => "aon",
		"pre" => ["aon_iram_begin"],
		"post" => ["aon_iram_end"],},
	".static_area" => {
		"sections" => [
			"KEEP(*(.data_in_static_area))"],
		"mem_type" => "static_section",
		"pre" => ["static_area_begin"],
		"post" => ["static_area_end"],},
	".pram_rodata" => {
		"sections" => [
			"KEEP(*(.dsp_pram_section))"],
		"mem_type" => "pram",
		"pre" => [],
        "post" => ["pram_area_end"],},
    ".pre_init_cfg" => {
        "sections" => [
			"KEEP(*(.pre_init_cfg))",
        ],
        "mem_type" => "ram_pre_init",
        "pre" => [],
		"post" => [],
	},
    ".app_entry" => {
        "align" => 32,
        "sections" => [
			"*(.app_entry)",
        ],
        "mem_type" => "ram",
        "pre" => ["app_entry_begin"],
		"post" => ["app_entry_end"],
	},
    ".log_section" => {
        "sections" => [
            "KEEP(*(log_data))"],
        "mem_type" => "log_section",
        "pre" => [],
		"post" => [],},
};

my $xip_extra_in = [
	"*(.app_init_code)",
	"*(.emb_text)",
	"*(.text)",
	"*(EmHeAacDec)",
	"*(.text.*)",
	"*(.gnu.linkonce.t.*)",
	"*(i.*)",
	"*(.glue_7t)",
	"*(.glue_7)",
	"*(.vfp11_veneer)",
	"*(.v4_bx)",
	"*(.init_code)",
	"*(.init_text)",
	"*(.spar_setup)",
	"*(.ARM.extab*)",
	"*(.gnu.linkonce.armextab.*)",
	"*(.ARM.exidx*)",
	"*(.gnu.linkonce.armexidx.*)",
	"*(const_drom_var)",
	"*(.rodata)",
	"*(.constdata*)",
	"*(.rodata.*)",
	"*(.gnu.linkonce.r.*)",
	"*(.data.rom)",
];

main();

sub main
{
	my $param;
#	print "args: @ARGS\n";

	foreach my $arg (@ARGV) {
		if($arg =~ /\.elf$/) {
			$param->{'elf'} = $arg;
		}
		if($arg =~ /\.sym$/) {
			$param->{'sym'} = $arg;
		}
		elsif($arg =~ /^ISTATIC_BEGIN=(\w+)/) {
			$param->{'ISTATIC_BEGIN'} = hex($1);
		}
		elsif($arg =~ /^ISTATIC_LEN=(\w+)/) {
			$param->{'ISTATIC_LEN'} = hex($1);
		}
		elsif($arg =~ /^UNUSED_BEGIN=(\w+)/) {
			$param->{'UNUSED_BEGIN'} = hex($1);
		}
		elsif($arg =~ /^UNUSED_LEN=(\w+)/) {
			$param->{'UNUSED_LEN'} = hex($1);
		}
		elsif($arg =~ /^AON_AREA_END=(\w+)/) {
			$param->{'AON_AREA_END'} = hex($1);
		}
		elsif($arg =~ /^SRAM_BEGIN_ADDR=(\w+)/) {
			$param->{'SRAM_BEGIN_ADDR'} = hex($1);
		}
		elsif($arg =~ /^SRAM_LENGTH=(\w+)/) {
			$param->{'SRAM_LENGTH'} = hex($1);
		}
        elsif($arg =~ /^PATCH_RAM_OBJ=(.*)/) {
            $param->{'PATCH_RAM_OBJ'} = $1;
            my @patch_ram_obj = split ";", $1;
            my @patch_ram_o;
            foreach my $obj (@patch_ram_obj) {
                $obj =~ s/^\./\*/;
                $obj .= " (.text.* .gnu.linkonce.t.* .rodata .constdata* .rodata.* .gnu.linkonce.r.*)";
                push @patch_ram_o, $obj;
		}
            unshift @{$mem_lut->{'.pram_rodata'}->{sections}}, @patch_ram_o;
		}
		elsif($arg =~ /^NUM_PATCH_ENTRIES=(\d+)/) {
			$param->{'NUM_PATCH_ENTRIES'} = $1;
		}
		elsif($arg =~ /^DIRECT_LOAD=(\d)/) {
			$param->{'direct_load'} = $1;
		}
		elsif($arg =~ /^LAYOUT=(\w+)/) {
			$param->{'layout'} = $1;
		}
		elsif($arg =~ /^BTP=(.*)/) {
			$param->{'btp'} = $1;
		}
		elsif($arg =~ /^FLASH0_BEGIN_ADDR=(\w+)/) {
			$param->{'FLASH0_BEGIN_ADDR'} = hex($1);
		}
		elsif($arg =~ /^FLASH0_LENGTH=(\w+)/) {
			$param->{'FLASH0_LENGTH'} = hex($1);
		}
		elsif($arg =~ /^XIP_DS_OFFSET=(\w+)/) {
			$param->{'XIP_DS_OFFSET'} = hex($1);
			$param->{'xip'} = 1;
			push @{$mem_lut->{'.app_xip_area'}->{sections}}, @{$xip_extra_in};
		}
		elsif($arg =~ /^XIP_LEN=(\w+)/) {
			$param->{'XIP_LEN'} = hex($1);
		}
		elsif($arg =~ /^XIP_OBJ=(.*)/) {
			my @xip_obj = split ";", $1;
			my @xip_o;
			foreach my $obj (@xip_obj) {
				$obj =~ s/^\./\*/;
				$obj .= " (.text.* .gnu.linkonce.t.* .rodata .constdata* .rodata.* .gnu.linkonce.r.*)";
				push @xip_o, $obj;
			}
			unshift @{$mem_lut->{'.app_xip_area'}->{sections}}, @xip_o;
		}
		elsif($arg =~ /^APP_DS2_LEN=(\w+)/) {
			$param->{'APP_DS2_LEN'} = hex($1);
		}
		elsif($arg =~ /^DS_LOCATION=(\w+)/) {
			$param->{'DS_LOCATION'} = hex($1);
		}
		elsif($arg =~ /^OTA_UPGRADE_STORE=(\w+)/) {
			$param->{'OTA_UPGRADE_STORE'} = $1;
		}
		elsif($arg =~ /^overlay=(.*)$/) {
			$param->{'overlay'} = $1;
			warn "overlay is $1\n";
		}
		elsif($arg =~ /^out=(.*)/) {
			$param->{'outfile'} = $1;
		}
	}

    if(defined $param->{btp} && defined $param->{FLASH0_BEGIN_ADDR} && defined $param->{FLASH0_LENGTH}) {
        open(my $BTP, "<", $param->{btp}) || die "Could not open *.btp file \"$param->{btp}\", $!";
        while(defined(my $line = <$BTP>)) {
            if($line =~ /\s*(\w+)\s*\=\s*(0x[0-9a-fA-F]+)/) {
                $param->{$1} = hex($2);
            }
            elsif($line =~ /\s*(\w+)\s*\=\s*([0-9]+)/) {
                $param->{$1} = int($2);
            }
        }
        close $BTP;
    }
    # override ConfigDSLocation if DS_LOCATION provided on command line
    if(defined $param->{ConfigDSLocation} && defined $param->{DS_LOCATION}) {
        $param->{ConfigDSLocation} = $param->{DS_LOCATION};
    }

    my $section_lut = {};
    if(defined $param->{elf}) {
        my $sections = [];
        my $stringtable = {};
        my $sym_str_tbl = {};
        my $symbol_entries = [];
        parse_elf($param->{elf}, $sections, $stringtable, $sym_str_tbl, $symbol_entries, 1);

        #printf "got %d sections\n", scalar(@{$sections});
        foreach my $section (@{$sections}) {
            if(!defined $section->{name}) {
                #print "section name index $section->{sh_name}\n";
                #printf("%s\n", $stringtable->{$section->{sh_name}}) if defined $section->{sh_name};
                $section->{name} = $stringtable->{$section->{sh_name}};
            }
            if(defined $section_lut->{$section->{name}}) {
                # if already defined, merge it's limits with previous (case for MPAF_SRAM_AREA)
                $section_lut->{$section->{name}}->{sh_addr} = $section->{sh_addr} if
                    $section_lut->{$section->{name}}->{sh_addr} > $section->{sh_addr};
                $section_lut->{$section->{name}}->{sh_size} = $section->{sh_size} if
                    $section_lut->{$section->{name}}->{sh_size} < $section->{sh_size};
            }
            else {
                $section_lut->{$section->{name}} = $section;
                #printf "section %s: start 0x%x len 0x%x\n", $section->{name}, $section->{sh_addr}, $section->{sh_size};
            }
        }
        my $gp_wiced_app_pre_init_cfg_sym = find_symbol($symbol_entries, "gp_wiced_app_pre_init_cfg");
        if(defined $gp_wiced_app_pre_init_cfg_sym)
        {
            $section_lut->{APP_PRE_INIT_CFG} = { sh_addr => $gp_wiced_app_pre_init_cfg_sym->{st_value} };
        }
    }
    elsif(defined $param->{sym}) {
        # using sym file, so fake reading section headers from elf
        open(my $SYM, "<", $param->{sym}) or die "Could not read $param->{sym}, $!\n";
        my $is_section_info_end = 0;
        while(defined(my $line = <$SYM>)) {
            if($line =~ /(\w+)\s*=\s*0x([0-9A-Fa-f]+)/) {
            my $name = $1;
                my $addr = hex($2);

                $is_section_info_end = 1 if $name eq 'END_SECTION_INFO';

                if (!$is_section_info_end) {
                    $section_lut->{$1} = { sh_addr => $addr };
                }
                elsif ($name eq 'gp_wiced_app_pre_init_cfg') {
                    $section_lut->{APP_PRE_INIT_CFG} = { sh_addr => $addr };
                }
            }
        }
        close $SYM;
    }
    $param->{SRAM_START_ADDR} = $section_lut->{first_free_section_in_SRAM}->{sh_addr}
            if defined $section_lut->{first_free_section_in_SRAM}->{sh_addr};
    $param->{SRAM_START_ADDR} = $section_lut->{FIRST_FREE_SECTION_IN_SRAM}->{sh_addr}
            if defined $section_lut->{FIRST_FREE_SECTION_IN_SRAM}->{sh_addr};
    # for DIRECT_LOAD, add app code after init sections (including SlimBoot)
    $param->{SRAM_START_ADDR} = $section_lut->{POST_INIT_SECTION_IN_SRAM}->{sh_addr}
            if defined $section_lut->{POST_INIT_SECTION_IN_SRAM}->{sh_addr} &&
                defined $param->{direct_load} && $param->{direct_load};
    die "Ram start undefined\n" if !defined $param->{SRAM_START_ADDR};
    output_ld($section_lut, $param);
}

sub output_ld
{
	my ($sections, $param) = @_;
	my ($ram_start, $ram_begin, $ram_end, $ram_len,
		$rom_start, $rom_begin, $rom_end, $rom_len,
		$aon_begin, $aon_end, $aon_len,
		$xip_start, $xip_len);

	if (defined $sections->{CODE_AREA}) {
        $rom_start = $sections->{CODE_AREA}->{sh_addr} if defined $sections->{CODE_AREA};
    }
    elsif (defined $sections->{PATCH_RAM_BASE}) {
        $rom_start = $sections->{PATCH_RAM_BASE}->{sh_addr} if defined $sections->{PATCH_RAM_BASE};
    }
    die "Could not locate rom start in patch elf\n" if !defined $rom_start;

    $ram_start = $param->{SRAM_START_ADDR};

	my $num_patches = $param->{NUM_PATCH_ENTRIES};
	$num_patches = 256 if !defined $num_patches;

	my $empty_mpaf_data_offset = 0;
	if($param->{layout} eq 'code_from_top') {
	    my $mpaf_data_area_section;
        if (defined $sections->{MPAF_SRAM_AREA}) {
            $mpaf_data_area_section = $sections->{MPAF_SRAM_AREA};
        }
        elsif (defined $sections->{mpaf_data_area}) {
            $mpaf_data_area_section = $sections->{mpaf_data_area};
        }
        elsif (defined $sections->{POST_MPAF_SECTION_IN_SRAM}) {
            $mpaf_data_area_section = $sections->{POST_MPAF_SECTION_IN_SRAM};
            $empty_mpaf_data_offset = 0x200;
        }
        else { # use end of sram for 43022, where mpaf data area is not defined
            $sections->{'MPAF_SRAM_AREA'} = { sh_addr => (0x251000 - ($num_patches * 4)), };
            $mpaf_data_area_section = $sections->{MPAF_SRAM_AREA};
            $empty_mpaf_data_offset = 0x200;
        }

        if (defined($param->{SRAM_LENGTH})) {
            # avoid loading right up to $mpaf_data_area_section
            $ram_start = $mpaf_data_area_section->{sh_addr} - $param->{SRAM_LENGTH} - 1 - $empty_mpaf_data_offset;
            # Round down to 32-byte
            $ram_start &= ~0x0000001f;
        }

        $ram_end = $mpaf_data_area_section->{sh_addr};
    }
    else {
        $ram_begin = $param->{SRAM_BEGIN_ADDR};
        $ram_end = $ram_begin + $param->{SRAM_LENGTH} - ($num_patches * 4);
        die "Could not locate data ram start in patch elf\n" if !defined $ram_begin;
    }
	die "Could not locate data ram end in patch elf\n" if !defined $ram_end;

	if (defined $sections->{FIRST_FREE_SECTION_IN_PROM}) {
		$rom_begin = $sections->{FIRST_FREE_SECTION_IN_PROM}->{sh_addr};
		$rom_end = $sections->{PATCH_CODE_END}->{sh_addr};
		die "Could not locate code ram start in patch elf\n" if !defined $rom_begin;
		die "Could not locate code ram end in patch elf\n" if !defined $rom_end;
	}

	if(defined $sections->{FIRST_FREE_SECTION_IN_AON}) {
		$aon_begin = $sections->{FIRST_FREE_SECTION_IN_AON}->{sh_addr};
		$aon_end = $sections->{AON_AREA_END}->{sh_addr};
		$aon_end = $param->{AON_AREA_END} if !defined $aon_end;
		die "Could not locate aon ram start in patch elf\n" if !defined $aon_begin;
		die "Could not locate aon ram end in patch elf\n" if !defined $aon_end;
	}

	my $patch_replace_area = $num_patches * 4;
	$ram_len = $ram_end - $ram_start - $patch_replace_area;
	$rom_len = $rom_end - $rom_begin if defined $rom_begin;
	$aon_len = $aon_end - $aon_begin if defined $aon_begin;

#	$(eval LINK_LOC_FLAGS += ISTATIC_BEGIN=$(PLATFORM_APP_SPECIFIC_STATIC_DATA) ISTATIC_LEN=$(PLATFORM_APP_SPECIFIC_STATIC_LEN) IRAM_BEGIN=$(IRAM_BEGIN) IRAM_LEN=$(IRAM_LEN) IROM_BEGIN=$(IROM_BEGIN) IROM_LEN=$(IROM_LEN) IAON_BEGIN=$(IAON_BEGIN) IAON_LEN=$(IAON_LEN) )
#	$(QUIET)$(XCC) -E -x c -P $(addprefix -D,$(LINK_LOC_FLAGS)) -I $(dir $(ELF_OUT)) -o $@ $<
	$param->{ISTATIC_BEGIN} = $param->{ISTATIC_BEGIN} if defined $param->{ISTATIC_BEGIN};
	$param->{ISTATIC_LEN} = $param->{ISTATIC_LEN} if defined $param->{ISTATIC_LEN};
	$param->{'direct_load'} = 0 if !defined $param->{direct_load};
	$param->{'layout'} = 'code_from_buttom' if !defined $param->{layout};

	open(my $OUT, ">", $param->{outfile}) || die "ERROR: Cannot open $param->{outfile}, $!";

	print $OUT "OUTPUT_FORMAT (\"elf32-littlearm\", \"elf32-bigarm\", \"elf32-littlearm\")\n";
	print $OUT "SEARCH_DIR(.)\n";
#	print $OUT "/* @ARGV */\n";
#	print $OUT "/*\n";
#	foreach my$key (sort(keys(%{$param}))) {
#	    print $OUT "$key => $param->{$key}\n";
#	}
#	print $OUT "*/\n";
	if(defined $rom_start) {
		print $OUT sprintf "/* pram_patch_begin=0x%06X pram_patch_end=0x%06X pram_end=0x%06X */\n", $rom_start, $rom_begin, $rom_end;
	}
	print $OUT sprintf "/* ram_patch_begin=0x%06X ram_patch_end=0x%06X ram_end=0x%06X */\n", $sections->{SRAM_AREA}->{sh_addr}, $ram_start, $ram_end;
	if(defined $aon_len) {
		print $OUT sprintf "/* aon_patch_begin=0x%06X aon_patch_end=0x%06X aon_end=0x%06X */\n", $sections->{AON_AREA}->{sh_addr}, $aon_begin, $aon_end;
	}
    if(defined $aon_len) {
        print $OUT sprintf "/* app_ram_begin=0x%06X app_ram_end=0x%06X */\n", $ram_start, $ram_start+$ram_len;
    }
	if(defined $param->{FLASH0_BEGIN_ADDR} && defined $param->{FLASH0_LENGTH}) {
		print $OUT sprintf "/* FLASH0_BEGIN_ADDR=0x%06X FLASH0_LENGTH=0x%06X */\n", $param->{FLASH0_BEGIN_ADDR}, $param->{FLASH0_LENGTH};
	}
	if(defined $param->{DLConfigSSLocation}) {
		print $OUT sprintf "/* FLASH0_SS=0x%06X */\n", $param->{DLConfigSSLocation};
	}
	if(defined $param->{DLConfigVSLocation}) {
		print $OUT sprintf "/* FLASH0_VS=0x%06X */\n", $param->{DLConfigVSLocation};
	}
	if(defined $param->{ConfigDSLocation}) {
		print $OUT sprintf "/* FLASH0_DS=0x%06X */\n", $param->{ConfigDSLocation};
	}
	if(defined $param->{ConfigDS2Location}) {
        if(defined $param->{APP_DS2_LEN}) {
            $param->{ConfigDS2Location} = $param->{FLASH0_BEGIN_ADDR} + $param->{FLASH0_LENGTH} - $param->{APP_DS2_LEN};
        }
		print $OUT sprintf "/* FLASH0_DS2=0x%06X */\n", $param->{ConfigDS2Location};
	}
    if(defined $param->{ConfigDSLocation}) {
        my $store = $param->{ConfigDS2Location} - $param->{ConfigDSLocation};
        $param->{'OTA_UPGRADE_STORE'} = 'off_chip_sflash' if !defined $param->{OTA_UPGRADE_STORE};
        $store /=2 if $param->{OTA_UPGRADE_STORE} eq 'on_chip_flash';
        print $OUT sprintf "/* UPGRADE_STORAGE_LENGTH=0x%06X (%s) */\n", $store, $param->{OTA_UPGRADE_STORE};
    }

	print $OUT "MEMORY\n";
	print $OUT "{\n";

    if (defined $sections->{APP_PRE_INIT_CFG}) {
        printf $OUT "\tram_pre_init (r) : ORIGIN = 0x%X, LENGTH = 0x%X\n", $sections->{APP_PRE_INIT_CFG}->{sh_addr}, 4;
    }
	print $OUT sprintf "\tram (rwx) : ORIGIN = 0x%X, LENGTH = 0x%X\n", $ram_start, $ram_end - $ram_start;
	if(defined $aon_len) {
		print $OUT sprintf "\taon (rwx) : ORIGIN = 0x%X, LENGTH = 0x%X\n", $aon_begin, $aon_len;
	}
	if(defined $param->{UNUSED_LEN}) {
		print $OUT sprintf "\tunused (rwx) : ORIGIN = 0x%X, LENGTH = %d\n", $param->{UNUSED_BEGIN}, $param->{UNUSED_LEN};
	}
	if(defined $param->{ISTATIC_BEGIN}) {
		print $OUT sprintf "\tstatic_section (r) : ORIGIN = 0x%X, LENGTH = 0x%X\n", $param->{ISTATIC_BEGIN}, $param->{ISTATIC_LEN};
	}
	if(defined $param->{xip}) {
		$xip_start = $param->{ConfigDSLocation} + $param->{XIP_DS_OFFSET};
		$xip_len = $param->{XIP_LEN};
		$xip_len = $param->{FLASH0_LENGTH} - ($xip_start - $param->{ConfigDSLocation}) if !defined $xip_len;
        print $OUT sprintf "\txip_section (rx) : ORIGIN = 0x%X, LENGTH = 0x%X\n", $xip_start, $xip_len;
	}
    if(defined $param->{PATCH_RAM_OBJ}) {
        print $OUT sprintf "\tpram (rwx) : ORIGIN = 0x%X, LENGTH = 0x%X\n", $rom_begin, $rom_end - $rom_begin;
	}
    print $OUT sprintf "\tlog_section (r) : ORIGIN = 0x81000004, LENGTH = 0x100000\n";
	print $OUT "}\n";

	print $OUT "EXTERN(aon_iram_end);\n" if defined $aon_len;
	print $OUT "EXTERN(xip_area_begin);\n" if defined $xip_start;
	print $OUT "EXTERN(xip_area_end);\n" if defined $xip_len;

	print $OUT "SECTIONS\n";
	print $OUT "{\n";

    if (defined $sections->{APP_PRE_INIT_CFG}) {
        output_section('.pre_init_cfg', $mem_lut, $OUT);
    }

	if($param->{layout} eq 'code_from_top') {
        output_section('.bss', $mem_lut, $OUT);

        print $OUT "\tapp_iram_data_begin = .;\n";
        output_ld_data($OUT);
        output_ld_code($OUT, $ram_start);
        print $OUT "\tapp_iram_data_end = .;\n";
        print $OUT "\tapp_iram_begin = app_iram_bss_begin;\n";
        printf $OUT "\tapp_iram_end = 0x%08x;\n", $ram_start + $ram_len;
    }
    else {
        output_ld_code($OUT, $ram_start);
        output_ld_data($OUT);
        print $OUT "\tapp_iram_data_end = .;\n";

        output_section('.bss', $mem_lut, $OUT);

        print $OUT "\tapp_iram_begin = app_entry_begin;\n";
        print $OUT "\tapp_iram_data_begin = app_entry_begin;\n";
        print $OUT "\tapp_iram_end = .;\n";
    }

    print $OUT "\tapp_iram_length = app_iram_end - app_iram_begin;\n";
    print $OUT "\tapp_iram_bss_length = app_iram_bss_end - app_iram_bss_begin;\n";
    print $OUT "\tapp_iram_data_length = app_iram_data_end - app_iram_begin;\n";
    print $OUT "\tapp_irom_data_begin = app_iram_bss_end;\n";

    output_section('.log_section', $mem_lut, $OUT);
	print $OUT "}\n";
}

sub output_ld_code
{
	my ($OUT, $ram_start) = @_;

	output_section('.app_entry', $mem_lut, $OUT);

#	print "\t#include \"spar_ram_overlays.ld\"\n";
	if(defined $param->{overlay}) {
		open(my $OVER, "<", $param->{overlay}) || die "Could not open overlay *.ld file \"$param->{overlay}\", $!";
		while(defined(my $line = <$OVER>)) {
			print $OUT $line;
		}
		close $OVER;
	}

	# if objects are assigned to XIP, match their .text and .rodata
	output_section('.app_xip_area', $mem_lut, $OUT) if defined $param->{xip};

	# When direct loading, don't overlap init code with dynamic allocation.
	if($param->{direct_load})
	{
		output_section('.setup', $mem_lut, $OUT);
	}
    output_section('.pram_rodata', $mem_lut, $OUT) if defined $param->{PATCH_RAM_OBJ};
    output_section('.text', $mem_lut, $OUT);
}

sub output_ld_data
{
	my ($OUT) = @_;

    output_section('.rodata', $mem_lut, $OUT);
	output_section('.data', $mem_lut, $OUT);

    if(!$param->{direct_load}) {
		# Place the setup area after bss so that when dynamic allocation occurs
		# after spar setup, it will reclaim the RAM taken up by the setup function.
		output_section('.setup', $mem_lut, $OUT);
	}
	output_section('.aon', $mem_lut, $OUT) if defined $aon_len;
	output_section('.static_area', $mem_lut, $OUT) if defined $param->{ISTATIC_BEGIN};
}

sub output_section
{
	my ($name, $lut, $fh, $pre, $post) = @_;
	my $info = $lut->{$name};
	die "Could not find ld table entry for $name\n" if !defined $info;

	my $align = "";
    if (defined($info->{align})) {
        $align = "ALIGN ($info->{align})" if ($info->{align} > 0);
    }
    else {
        $align = "ALIGN (4)";
    }

	print $fh "\t$name : $align\n\t{\n";
	print $fh "\t\tCREATE_OBJECT_SYMBOLS\n";
	foreach my $sym (@{$info->{pre}}) {
		print $fh "\t\t$sym = .;\n";
	}
	foreach my $sect (@{$info->{sections}}) {
		print $fh "\t\t$sect\n";
	}
	foreach my $sym (@{$info->{post}}) {
		print $fh "\t\t$sym = .;\n";
	}
	print $fh "\t} >$info->{mem_type}\n";
#	print "\n";
}
