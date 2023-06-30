#!/usr/bin/perl
#
# Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
# wiced-split-ss-ds-hex.pl hex_in=<*.hex> [(hex_ss|hex_ds|hex_dl|bin_ss|bin_ds|bin_dl)=<file>]
#                          [ss_bda=<6 byte hex>] - overwrite BT DEVICE ADDRESS data in any SS output
#                          [ss_dsa=<ds addr hex>] - overwrite ds address in ss layout record\
#                          [ds_app] - switch BRCMcfgD signature to BRCLcfgA\
# read in hex records from hex_in=<> file arg, sort ss from ds from dl (direct load)
# process arguments to determine output files, hex or binary
# optionally do further processing to:
#     output address of start SS or DS (at ^BRCMcfgS or ^BRCMcfgD) plus length + 16 (header) + 64 (signature)
#     output address of end SS or DS
#     modify TypeLengthValue data in SS to change BDADDR or DS address
use warnings;
use strict;

main();

sub main
{
    my @hex_records;
    my @ss_hex_records;
    my @ds_hex_records;
    my @all_hex_records;
    my @dl_hex_records;
    my $base_addr = 0;
    my ($ss_addr_last, $ds_addr_last);
    my ($ss_addr, $ds_addr);
    my ($ss_len, $ds_len);
    my $param = {};
    my $valid_params = {
        hex_in => 0,
        hex_ss => 0,
        hex_ds => 0,
        hex_dl => 0,
        bin_ss => 0,
        bin_ds => 0,
        bin_dl => 0,
        ds_app => 0,
        ss_bda => 0,
        ss_dsa => 0,
        ss_end => 0,
        ds_end => 0,
        ss_start => 0,
        ds_start => 0,
    };

    foreach my $arg (@ARGV) {
        #print "# ----- $arg\n";
        if($arg =~ /(\w+)=(.*)/) {
            $param->{$1} = $2;
        }
        elsif(defined $valid_params->{$arg}) {
            $param->{$arg} = 1;
        }
        else {
            die "$arg is not a valid parameter\n";
        }
    }

    if(!defined $param->{hex_in}) {
        print "Use script to split hex file into ss, ds, or dl.\n";
        print "$0 hex_in=<*.hex> [(hex_ss|hex_ds|hex_dl|bin_ss|bin_ds|bin_dl)=<file>]\n";
        print "[ss_bda=<6 byte hex>] - overwrite BT DEVICE ADDRESS data in any SS output\n";
        print "[ss_dsa=<ds addr hex>] - overwrite ds address in ss layout record\n";
        print "[ds_app] - switch BRCMcfgD signature to BRCLcfgA\n";
        die "[ds_end] - print address at DS+signature end\n";
    }
    else {
        die "Could not find file \"$param->{hex_in}\"\n" unless -e $param->{hex_in};
    }
    if(defined $param->{ss_bda}) {
        die "ss_bda=<addr> address must be 12 hex characters\n" if $param->{ss_bda} !~ /[0-9a-fA-F]{12}/;
    }

    # read in records from first hex
    open(my $HEX_IN, "<", $param->{hex_in}) || die "ERROR: Cannot open $param->{hex_in}, $!";
    while(defined(my $line = <$HEX_IN>)) {
        my $record = {};
        $line =~ s/^\://;
        $line =~ s/\s+$//;
        # print "line = $line\n";
        hex2record($line, $record);
        if($record->{type} == 4) {
            my ($addr) = unpack "n", $record->{data};
            $base_addr = $addr << 16; # keep high 16 addr base
        }
        if($record->{type} == 0) {
            $record->{'full_addr'} = $record->{addr} + $base_addr; # get 32-bit addr
            push @all_hex_records, $record;
        }
        if($record->{len} > (8+8)) { # header, text
            my $signature = substr($record->{data}, 0, 8);
            # dump_bytes($record->{data});
            # printf "signature $signature, length %d\n", length($signature) ;
            if($signature eq "BRCMcfgS") {
                $ss_addr = $record->{full_addr};
                ($ss_len) = unpack "L", substr($record->{data}, 12, 4);
                $ss_addr_last = $record->{full_addr} + $ss_len + 16;
            }
            elsif($signature eq "BRCMcfgD")
            {
                $ds_addr = $record->{full_addr};
                ($ds_len) = unpack "L", substr($record->{data}, 12, 4);
                $ds_addr_last = $record->{full_addr} + $ds_len + 16 + 64;
                # change BRCMcfgD to BRCMcgfA
                $record->{data} =~ s/BRCMcfgD/BRCMcfgA/ if $param->{ds_app};
            }
        }
    }
    close $HEX_IN;

  #  printf "ss start %08x end %08x len %d\n", $ss_addr, $ss_addr_last, $ss_len;
  #  printf "ds start %08x end %08x len %d\n", $ds_addr, $ds_addr_last, $ds_len;

    # sort the records into ss, ds, and dl (assume no one record is both SS and DS)
    foreach my $record (@all_hex_records) {
        if( ($record->{full_addr} >= $ss_addr) && ($record->{full_addr} < $ss_addr_last) ) {
            push @ss_hex_records, $record;
           # printf "ss %08x %d bytes\n", $record->{full_addr}, $record->{len};
        }
        elsif( ($record->{full_addr} >= $ds_addr) && ($record->{full_addr} < $ds_addr_last) ) {
            push @ds_hex_records, $record;
           # printf "ds %08x %d bytes\n", $record->{full_addr}, $record->{len};
        }
        else {
            push @dl_hex_records, $record;
           # printf "dl %08x %d bytes\n", $record->{full_addr}, $record->{len};
        }
    }

    # update BT DEVICE ADDRESS
    if(defined $param->{ss_bda}) {
        my ($bda) = pack "H*", $param->{ss_bda};
        update_ss_tlv(0x300, 6, $bda, \@ss_hex_records);
    }

    # update DS address in SS record
    if(defined $param->{ss_dsa}) {
        my $dsa = pack "L", hex($param->{ss_dsa});
        update_ss_tlv(0x100, 0x1c, $dsa, \@ss_hex_records);
    }

    # generate output files
    if(defined $param->{hex_ss}) {
        output_hex($param->{hex_ss}, \@ss_hex_records, 1);
    }
    if(defined $param->{hex_ds}) {
        output_hex($param->{hex_ds}, \@ds_hex_records);
    }
    if(defined $param->{hex_dl}) {
        output_hex($param->{hex_dl}, \@dl_hex_records);
    }
    if(defined $param->{bin_ss}) {
        output_bin($param->{bin_ss}, \@ss_hex_records);
    }
    if(defined $param->{bin_ds}) {
        output_bin($param->{bin_ds}, \@ds_hex_records);
    }
    if(defined $param->{bin_dl}) {
        output_bin($param->{bin_dl}, \@dl_hex_records);
    }

    if($param->{ds_end}) {
        printf "ds_end=0x%08X ", $ds_addr_last;
    }
    if($param->{ss_end}) {
        printf "ss_end=0x%08X ", $ss_addr_last;
    }
    if($param->{ss_start}) {
        printf "ss_start=0x%08X ", $ss_addr;
    }
    if($param->{ds_start}) {
        printf "ds_start=0x%08X ", $ds_addr;
    }
    print "\n";
}

sub output_hex
{
    my ($file, $records) = @_;
    open(my $HEX, '>', "$file") or die "Cannot open $file for write: $!\n";
    binmode $HEX;
    my $addr_high = 0;
    foreach my $record (@{$records}) {
        if(($record->{full_addr} >> 16) != $addr_high) {
            $addr_high = $record->{full_addr} >> 16;
            print $HEX hex_record(0, 4, sprintf("%04X", $addr_high)), "\n";
        }
        # printf "%s  // %s\n", $record->{text}, substr($record->{text}, 8, $record->{len}*2);
        print $HEX hex_record($record->{addr}, $record->{type}, substr($record->{text}, 8, $record->{len}*2)), "\n";
    }
    print $HEX hex_record(0, 1, ""), "\n";
    close $HEX;
}

sub output_bin
{
    my ($file, $records) = @_;
    # go binary too...
    open(my $BIN, '>', "$file") or die "Cannot open $file for write: $!\n";
    binmode $BIN;
    foreach my $record (@{$records}) {
        print $BIN $record->{data};
    }
    close $BIN;
}

sub hex2record
{
    my ($data, $record) = @_;

    # print "unpack $data\n";
    $record->{'text'} = $data;
    $data = pack("H*", $data);
    (	$record->{'len'},
        $record->{'addr'},
        $record->{'type'}) = unpack("CnC", $data);
    ($record->{'checksum'}) = unpack("C", substr($data,-1));
    # printf "len 0x%02X addr 0x%04X type %d\n", $record->{'len'}, $record->{'addr'}, $record->{'type'};
    $record->{'data'} = substr($data,4,$record->{len});
}

sub hex_record
{
    my ($addr, $type, $data) = @_;
    die "record must be an even number of hex digits\n" if length($data) & 1;
    my $record = sprintf "%02X%04X%02X", length($data)/2, $addr, $type;
    $record .= uc($data);
 #   print "$record\n";
    my $checksum = 0;
    $checksum += $_ for unpack('C*', pack("H*", $record));
    my $hex_sum = sprintf "%04X", $checksum;
    $hex_sum = substr($hex_sum, -2); # just save the last byte of sum
    $checksum = (hex($hex_sum) ^ 0xFF) + 1; # 2's complement of hex_sum
    $checksum &= 0xff;
    $checksum = sprintf "%02X", $checksum; # convert checksum to string
    $record = ":" . $record . $checksum;
    return $record;
}

sub update_ss_tlv
{
    # tlv type and length are used to identify the tlv to modify
    # the data is replaced starting from the beginning of the tlv value field
    my ($tlv_type, $tlv_len, $new_data, $ss_records) = @_;
    my $data;
    # collect ss binary
    foreach my $record (@{$ss_records}) {
        $data .= $record->{data};
    }
    # skip signature to get to tlv
    my $offset = 16;
    # search and replace bda
    while($offset < length($data)) {
        my ($type) = unpack "S", substr($data, $offset, 2);
        $offset += 2;
        my ($len) = unpack "C", substr($data, $offset, 1);
        $offset += 1;
        if($type == $tlv_type && $len == $tlv_len) {
            substr($data, $offset, length($new_data), $new_data);
        }
        $offset += $len;
    }
    # rebuild ss records
    $offset = 0;
    foreach my $record (@{$ss_records}) {
        next if $record->{data} eq substr($data, $offset, $record->{len});
        my $hex_text;
        my @bytes = unpack "C*", substr($data, $offset, $record->{len});
        foreach my $byte (@bytes) {
            $hex_text .= sprintf "%02X", $byte;
        }
        my $r = {};
        my $line = hex_record($record->{addr}, $record->{type}, $hex_text);
        $line =~ s/^\://;
        hex2record($line, $r);
        $record->{data} = $r->{data};
        $record->{text} = $r->{text};
        $record->{checksum} = $r->{checksum};
        $offset += $record->{len};
    }
}

sub dump_bytes
{
    my ($data) = @_;
    my @bytes = unpack "C*", $data;
    # printf "dump %d bytes, %d\n", scalar(@bytes), length($data);
    my $count = 0;
    foreach my $byte (@bytes) {
        printf "%02x ", $byte;
        $count++;
        print "\n" if 0 == ($count % 16);
    }
}
