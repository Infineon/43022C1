#!/bin/bash
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
(set -o igncr) 2>/dev/null && set -o igncr; # this comment is required
set -e
#set -x

#######################################################################################################################
# This script performs post-build operations to form Bluetooth application download images.
#
# usage:
#   bt_post_build.bash  --shell=<modus shell path>
#                       --cross=<cross compiler path>
#                       --scripts=<wiced scripts path>
#                       --tools=<wiced tools path>
#                       --builddir=<mainapp build dir>
#                       --elfname=<app elf>
#                       --appname=<app name>
#                       --appver=<app id and major/minor version>
#                       --hdf=<hdf file>
#                       --entry=<entry function name>
#                       --cgslist=<cgs file list>
#                       --failsafe=<failsafe cgs file>
#                       --cgsargs=<cgs tool args>
#                       --btp=<btp file>
#                       --id=<hci id>
#                       --overridebaudfile=<override baud rate list>
#                       --chip=<chip>
#                       --target=<target>
#                       --minidriver=<minidriver file>
#                       --clflags=<chipload tool flags>
#                       --extrahex=<hex file to merge>
#                       --extras=<extra build actions>
#                       --verbose
#
#######################################################################################################################
USAGE="(-s=|--shell=)<shell path> (-x=|--cross=)<cross tools path>"
USAGE+=" (-w=|--scripts=)<wiced scripts path> (-t=|--tools=)<wiced tools>"
USAGE+=" (-b=|--builddir=)<build dir> (-e=|--elfname=)<elf name>"
USAGE+=" (-a=|--appname=)<app name> (-u|--appver=)<app version> (-d=|--hdf=)<hdf file>"
USAGE+=" (-n=|--entry=)<app entry function> (-l=|--cgslist=)<cgs file list>"
USAGE+=" (-z=|--sscgs=)<static cgs file> -f=|--failsafe=)<failsafe cgs file> (-g=|--cgsargs=)<cgs tool arg>"
USAGE+=" (-p=|--btp=)<btp file> (-i=|--id=)<hci id file> (-o=|--overridebaudfile=)<override baud rate list>"
USAGE+=" (-q=|--chip=)<chip> (-r=|--target=)<target>"
USAGE+=" (-m=|--minidriver=)<minidriver file> (-c=|--clflags=)<chipload tool flags>"
USAGE+=" (-j=|--extrahex=)<hex file to merge> (-k=|--extras=)<extra build action>"

if [[ $# -eq 0 ]]; then
    echo "usage: $0 $USAGE"
    exit 1
fi

VERBOSE=0

for i in "$@"
do
    case $i in
    -s=*|--shell=*)
        CYMODUSSHELL="${i#*=}"
        shift
        ;;
    -x=*|--cross=*)
        CYCROSSPATH="${i#*=}"
        CYCROSSPATH=${CYCROSSPATH//\\/\/}
        shift
        ;;
    -w=*|--scripts=*)
        CYWICEDSCRIPTS="${i#*=}"
        CYWICEDSCRIPTS=${CYWICEDSCRIPTS//\\/\/}
        shift
        ;;
    -t=*|--tools=*)
        CYWICEDTOOLS="${i#*=}"
        CYWICEDTOOLS=${CYWICEDTOOLS//\\/\/}
        shift
        ;;
    -b=*|--builddir=*)
        CY_MAINAPP_BUILD_DIR="${i#*=}"
        CY_MAINAPP_BUILD_DIR=${CY_MAINAPP_BUILD_DIR//\\/\/}
        shift
        ;;
    -e=*|--elfname=*)
        CY_ELF_NAME="${i#*=}"
        shift
        ;;
    -a=*|--appname=*)
        CY_MAINAPP_NAME="${i#*=}"
        shift
        ;;
    -u=*|--appver=*)
        CY_MAINAPP_VERSION="${i#*=}"
        shift
        ;;
    -d=*|--hdf=*)
        CY_APP_HDF="${i#*=}"
        shift
        ;;
    -n=*|--entry=*)
        CY_APP_ENTRY="${i#*=}"
        shift
        ;;
    -l=*|--cgslist=*)
        CY_APP_CGSLIST="${i#*=}"
        shift
        ;;
    -f=*|--failsafe=*)
        CY_APP_FAILSAFE="${i#*=}"
        shift
        ;;
    -z=*|--sscgs=*)
        CY_APP_SS_CGS="${i#*=}"
        shift
        ;;
    -g=*|--cgsargs=*)
        CY_APP_CGS_ARGS="${i#*=}"
        shift
        ;;
    -p=*|--btp=*)
        CY_APP_BTP="${i#*=}"
        shift
        ;;
    -i=*|--id=*)
        CY_APP_HCI_ID="${i#*=}"
        shift
        ;;
    -o=*|--overridebaudfile=*)
        CY_APP_BAUDRATE_FILE="${i#*=}"
        shift
        ;;
    -q=*|--chip=*)
        CY_CHIP="${i#*=}"
        shift
        ;;
    -r=*|--target=*)
        CY_TARGET="${i#*=}"
        shift
        ;;
    -m=*|--minidriver=*)
        CY_APP_MINIDRIVER="${i#*=}"
        shift
        ;;
    -2=*|--ds2_app=*)
        CY_DS2_APP_HEX="${i#*=}"
        shift
        ;;
    -c=*|--clflags=*)
        CY_APP_CHIPLOAD_FLAGS="${i#*=}"
        shift
        ;;
    -j=*|--extrahex=*)
        CY_PATCH_SIGNED_HEX="${i#*=}" # new for 43022C1
        shift
        ;;
    -k=*|--extras=*)
        CY_APP_BUILD_EXTRAS="${i#*=}"
        shift
        ;;
    --patch=*)
        CY_APP_PATCH="${i#*=}"
        shift
        ;;
    --ldargs=*)
        CY_APP_LD_ARGS="${i#*=}"
        shift
        ;;
    -v|--verbose)
        VERBOSE=1
        shift
        ;;
    -h|--help)
        HELP=1
        echo "usage: $0 $USAGE"
        exit 1
        ;;
    *)
        echo "bad parameter $i"
        echo "usage: $0 $USAGE"
        echo "failed to generate download file"
        exit 1
        ;;
    esac
done
CY_APP_HCD=

echo "Begin post build processing"
if [ ${VERBOSE} -ne 0 ]; then
    echo 1:  CYMODUSSHELL         : $CYMODUSSHELL
    echo 2:  CYCROSSPATH          : $CYCROSSPATH
    echo 3:  CYWICEDSCRIPTS       : $CYWICEDSCRIPTS
    echo 4:  CYWICEDTOOLS         : $CYWICEDTOOLS
    echo 5:  CY_MAINAPP_BUILD_DIR : $CY_MAINAPP_BUILD_DIR
    echo 6:  CY_ELF_NAME          : $CY_ELF_NAME
    echo 7:  CY_MAINAPP_NAME      : $CY_MAINAPP_NAME
    echo 7:  CY_MAINAPP_VERSION   : $CY_MAINAPP_VERSION
    echo 8:  CY_APP_HDF           : $CY_APP_HDF
    echo 9:  CY_APP_ENTRY         : $CY_APP_ENTRY
    echo 10: CY_APP_CGSLIST       : $CY_APP_CGSLIST
    echo 11: CY_APP_SS_CGS        : $CY_APP_SS_CGS
    echo 12: CY_APP_CGS_ARGS      : $CY_APP_CGS_ARGS
    echo 13: CY_APP_BTP           : $CY_APP_BTP
    echo 14: CY_APP_HCI_ID        : $CY_APP_HCI_ID
    echo 15: CY_APP_BAUDRATE_FILE : $CY_APP_BAUDRATE_FILE
    echo 16: CY_CHIP              : $CY_CHIP
    echo 17: CY_TARGET            : $CY_TARGET
    echo 18: CY_APP_MINIDRIVER    : $CY_APP_MINIDRIVER
    echo 19: CY_APP_CHIPLOAD_FLAGS: $CY_APP_CHIPLOAD_FLAGS
    echo 20: CY_PATCH_SIGNED_HEX  : $CY_PATCH_SIGNED_HEX
    echo 21: CY_APP_BUILD_EXTRAS  : $CY_APP_BUILD_EXTRAS
    echo 22: CY_APP_PATCH         : $CY_APP_PATCH
    echo 23: CY_APP_LD_ARGS       : $CY_APP_LD_ARGS
fi

# check that required files are present
if [ ! -e "$CY_MAINAPP_BUILD_DIR/$CY_ELF_NAME" ]; then
    echo "$CY_MAINAPP_BUILD_DIR/$CY_ELF_NAME not found!"
    exit 1
fi

CY_APP_HEX="$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}_download.hex"
CY_APP_HEX_STATIC="$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}_static.hex"
CY_APP_HEX_SS="$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}_ss.hex"
CY_APP_HCD="$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}_download.hcd"
CY_APP_LD="$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}_postbuild.ld"
CY_APP_MAP="$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}.map"
CY_APP_CGS_MAP="$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}.cgs.map"
CY_APP_SIGNED_HEX_BASE=$(basename ${CY_PATCH_SIGNED_HEX})
CY_APP_SIGNED_HEX="$CY_MAINAPP_BUILD_DIR/$CY_APP_SIGNED_HEX_BASE"

# check dependencies - only rebuild when needed
if [ -e "$CY_APP_HEX" ]; then
    echo "hex file already exists"
    if [ "$CY_APP_HEX" -nt "$CY_MAINAPP_BUILD_DIR/$CY_ELF_NAME" ]; then
      echo
      echo "hex file is newer than elf, skipping post-build operations"
      echo "make clean to refresh hex if needed"
      echo
      exit 0
    fi
fi

# set up some tools that may be native and not modus-shell
CY_TOOL_WC=wc
CY_TOOL_SYNC=sync
CY_TOOL_MV=mv
CY_TOOL_CAT=cat
CY_TOOL_PERL=perl
CY_TOOL_RM=rm
CY_TOOL_CP=cp
CY_TOOL_ECHO=echo
CY_TOOL_TAIL=tail
if ! type "$CY_TOOL_WC" &> /dev/null; then
CY_TOOL_WC=$CYMODUSSHELL/bin/wc
fi
if ! type "$CY_TOOL_SYNC" &> /dev/null; then
CY_TOOL_SYNC=$CYMODUSSHELL/bin/sync
fi
if ! type "$CY_TOOL_MV" &> /dev/null; then
CY_TOOL_MV=$CYMODUSSHELL/bin/mv
fi
if ! type "$CY_TOOL_CAT" &> /dev/null; then
CY_TOOL_CAT=$CYMODUSSHELL/bin/cat
fi
if ! type "$CY_TOOL_PERL" &> /dev/null; then
CY_TOOL_PERL=$CYMODUSSHELL/bin/perl
fi
if ! type "$CY_TOOL_RM" &> /dev/null; then
CY_TOOL_RM=$CYMODUSSHELL/bin/rm
fi
if ! type "$CY_TOOL_CP" &> /dev/null; then
CY_TOOL_CP=$CYMODUSSHELL/bin/cp
fi
if ! type "$CY_TOOL_ECHO" &> /dev/null; then
CY_TOOL_ECHO=$CYMODUSSHELL/bin/echo
fi
if ! type "$CY_TOOL_TAIL" &> /dev/null; then
CY_TOOL_TAIL=$CYMODUSSHELL/bin/tail
fi

# clean up any previous copies
"$CY_TOOL_RM" -f "$CY_MAINAPP_BUILD_DIR/configdef*.hdf" "$CY_APP_HCD" "$CY_APP_HEX" "$CY_MAINAPP_BUILD_DIR/$CY_MAINAPP_NAME.cgs" "$CY_MAINAPP_BUILD_DIR/det_and_id.log" "$CY_MAINAPP_BUILD_DIR/download.log"

if [[ $CY_APP_BUILD_EXTRAS = *"DIRECT"* ]]; then
    echo "building image for direct ram load (*.hcd)"
    CY_APP_DIRECT_LOAD_ADDR=$(echo $CY_APP_CGS_ARGS | "$CY_TOOL_PERL" -ple 's/.*DLConfigSSLocation:(0x[0-9A-Fa-f]+).*/$1/')
    #echo "Direct load adress $CY_APP_DIRECT_LOAD_ADDR"
    CY_APP_DIRECT_LOAD="DIRECT_LOAD=$CY_APP_DIRECT_LOAD_ADDR"
    CY_APP_CGS_ARGS+=" -O \"DLConfigTargeting:RAM runtime\""
    CY_APP_CGS_ARGS+=" -O DLConfigVSLocation:0"
    CY_APP_CGS_ARGS+=" -O DLConfigVSLength:0"
    CY_APP_CGS_ARGS+=" -O DLWriteVerifyMode:Write"
    CY_APP_CGS_ARGS+=" -O \"DLSectorEraseMode:Written sectors only\""
fi

# generate the ld script
APP_IRAM_LENGTH=$("${CY_TOOL_PERL}" -ne 'print "$1" if /(0x[0-9a-f]+)\s+app_iram_length/' "${CY_APP_MAP}")
GEN_LD_COMMAND="\
    ${CY_TOOL_PERL} -I ${CYWICEDSCRIPTS} ${CYWICEDSCRIPTS}/wiced-gen-ld.pl\
    ${CY_APP_DIRECT_LOAD}\
    ${CY_APP_PATCH}\
    BTP=${CY_APP_BTP}\
    LAYOUT=code_from_top\
    SRAM_LENGTH=${APP_IRAM_LENGTH}\
    NUM_PATCH_ENTRIES=192\
    out=${CY_APP_LD}"
if [ ${VERBOSE} -ne 0 ]; then
    echo Calling ${GEN_LD_COMMAND}
fi
set +e
eval ${GEN_LD_COMMAND}
set -e

# link
LD_COMMAND="\
    ${CYCROSSPATH}gcc\
    -o ${CY_MAINAPP_BUILD_DIR}/${CY_ELF_NAME}\
    -T${CY_APP_LD}\
    -Wl,-Map=${CY_APP_MAP/.map/_postbuild.map}\
    -Wl,--entry=${CY_APP_ENTRY}\
    -Wl,--just-symbols=${CY_APP_PATCH}\
    ${CY_APP_LD_ARGS}"
if [ ${VERBOSE} -ne 0 ]; then
    echo Calling ${LD_COMMAND}
fi
set +e
eval ${LD_COMMAND}
set -e

# generate asm listing
"${CYCROSSPATH}objdump" --disassemble "$CY_MAINAPP_BUILD_DIR/$CY_ELF_NAME" > "$CY_MAINAPP_BUILD_DIR/${CY_ELF_NAME/elf/asm}"

#create app cgs file
CREATE_CGS_COMMAND="\
    ${CY_TOOL_PERL} -I ${CYWICEDSCRIPTS} ${CYWICEDSCRIPTS}/wiced-gen-cgs.pl\
    ${CY_MAINAPP_BUILD_DIR}/${CY_ELF_NAME}\
    ${CY_APP_DIRECT_LOAD}\
    ${CY_APP_CGSLIST}\
    ${CY_APP_HDF}\
    ${CY_APP_LD}\
    ${CY_APP_BTP}\
    ${CY_APP_ENTRY}\
    out=${CY_MAINAPP_BUILD_DIR}/${CY_MAINAPP_NAME}.cgs > ${CY_MAINAPP_BUILD_DIR}/${CY_MAINAPP_NAME}.report.txt"
if [ ${VERBOSE} -ne 0 ]; then
    echo Calling ${CREATE_CGS_COMMAND}
fi
set +e
eval ${CREATE_CGS_COMMAND}
set -e
CY_RESOURCE_REPORT=$CY_MAINAPP_BUILD_DIR/$CY_MAINAPP_NAME.report.txt
"$CY_TOOL_CAT" "${CY_RESOURCE_REPORT}"

# copy hdf local for cgs tool, it seems to need it despite -D
"$CY_TOOL_CP" "$CY_APP_HDF" "$CY_MAINAPP_BUILD_DIR/."

# set up BDADDR if random or default
CY_APP_CGS_ARGS_ORIG=$CY_APP_CGS_ARGS
CY_APP_CGS_ARGS=$("$CY_TOOL_PERL" -I "$CYWICEDSCRIPTS" "$CYWICEDSCRIPTS/wiced-bdaddr.pl" ${CY_CHIP} "$CY_APP_BTP" ${CY_APP_CGS_ARGS})

# add ss cgs if needed, copy local to use local hdf
if [ "$CY_APP_SS_CGS" != "" ]; then
"$CY_TOOL_CP" "$CY_APP_SS_CGS" "$CY_MAINAPP_BUILD_DIR/."
CY_APP_SS_CGS=$(basename $CY_APP_SS_CGS)
CY_APP_SS_CGS="--ss-cgs \"$CY_MAINAPP_BUILD_DIR/$CY_APP_SS_CGS\""
fi

# generate hex download file, use eval because of those darn quotes needed around "DLConfigTargeting:RAM runtime"
# use set +e because of the darn eval
echo "cgs -D $CY_MAINAPP_BUILD_DIR $CY_APP_CGS_ARGS -B $CY_APP_BTP -P $CY_MAINAPP_VERSION -I $CY_APP_HEX -H $CY_APP_HCD $CY_APP_SS_CGS -M $CY_APP_CGS_MAP --cgs-files $CY_MAINAPP_BUILD_DIR/$CY_MAINAPP_NAME.cgs"
set +e
eval "$CYWICEDTOOLS/CGS/cgs -D $CY_MAINAPP_BUILD_DIR $CY_APP_CGS_ARGS -B $CY_APP_BTP -P $CY_MAINAPP_VERSION -I $CY_APP_HEX -H $CY_APP_HCD $CY_APP_SS_CGS -M $CY_APP_CGS_MAP --cgs-files $CY_MAINAPP_BUILD_DIR/$CY_MAINAPP_NAME.cgs"
set -e
if [[ ! -e $CY_APP_HEX ]]; then
    echo "!! Post build failed, no hex file output"
    exit 1
fi

"$CY_TOOL_CP" "${CY_PATCH_SIGNED_HEX}" "${CY_APP_SIGNED_HEX}"

# split up the signed patch hex to merge with app SS & DS & DIRECT_LOAD
SPLIT_PATCH_HEX_COMMAND="${CY_TOOL_PERL} ${CYWICEDSCRIPTS}/wiced-split-ss-ds-hex.pl "
SPLIT_PATCH_HEX_COMMAND+="hex_in=${CY_APP_SIGNED_HEX} "
SPLIT_PATCH_HEX_COMMAND+="hex_ds=${CY_APP_SIGNED_HEX}.ds.hex "
SPLIT_PATCH_HEX_COMMAND+="hex_dl=${CY_APP_SIGNED_HEX}.dl.hex "
SPLIT_PATCH_HEX_COMMAND+="ss_start "
SPLIT_PATCH_HEX_COMMAND+="ds_start "
SPLIT_PATCH_HEX_COMMAND+="ds_end"
if [ ${VERBOSE} -ne 0 ]; then
    echo Calling ${SPLIT_PATCH_HEX_COMMAND}
fi
# note position for app ds to start when appending
split_patch_response=$(${SPLIT_PATCH_HEX_COMMAND})
if [ ${VERBOSE} -ne 0 ]; then
    echo "wiced-split-ss-ds-hex.pl response: $split_patch_response"
fi
for key_val in $split_patch_response; do
    case $key_val in
    ds_end=*)
        patch_ds_end=${key_val#*=}
        ;;
    ds_start=*)
        patch_ds_start=${key_val#*=}
        ;;
    ss_start=*)
        patch_ss_start=${key_val#*=}
        ;;
    *)
        echo "bad parameter $key_val in wiced-split-ss-ds-hex.pl response"
        exit 1
        ;;
    esac
done

# check that this completed
if [[ ! -e ${CY_APP_SIGNED_HEX}.ds.hex ]]; then
    echo "!! Post build failed, patch hex file ${CY_APP_SIGNED_HEX}.ds.hex was not created"
    exit 1
fi

# split off the DIRECT_LOAD entries to a separate cgs file to convert to hex and merge with SS & DS
SPLIT_APP_HEX_COMMAND="${CY_TOOL_PERL} ${CYWICEDSCRIPTS}/wiced-split-ss-ds-hex.pl "
SPLIT_APP_HEX_COMMAND+="hex_in=${CY_APP_HEX} "
SPLIT_APP_HEX_COMMAND+="bin_ds=${CY_APP_HEX}.ds.bin "
SPLIT_APP_HEX_COMMAND+="hex_ss=${CY_APP_HEX}.ss.hex "
SPLIT_APP_HEX_COMMAND+="hex_dl=${CY_APP_HEX}.dl.hex "
SPLIT_APP_HEX_COMMAND+="ds_app "
SPLIT_APP_HEX_COMMAND+="ss_dsa=$patch_ds_start"
if [ ${VERBOSE} -ne 0 ]; then
    echo Calling ${SPLIT_APP_HEX_COMMAND}
fi
set +e
eval ${SPLIT_APP_HEX_COMMAND}
set -e
# check that this completed
if [[ ! -e ${CY_APP_HEX}.ss.hex ]]; then
    echo "!! Post build failed, application hex file ${CY_APP_HEX}.ss.hex was not created"
    exit 1
fi

# merge app SS with patch DS
MERGE_PATCH_HEX_COMMAND="${CYWICEDTOOLS}/IntelHexToBin/IntelHexMerge "
MERGE_PATCH_HEX_COMMAND+="-L 240 ${CY_APP_HEX}.ss.hex ${CY_APP_SIGNED_HEX}.ds.hex ${CY_APP_SIGNED_HEX}.ss_ds.hex"
if [ ${VERBOSE} -ne 0 ]; then
    echo Calling ${MERGE_PATCH_HEX_COMMAND}
fi
set +e
eval ${MERGE_PATCH_HEX_COMMAND}
set -e
# check that this completed
if [[ ! -e ${CY_APP_SIGNED_HEX}.ss_ds.hex ]]; then
    echo "!! Post build failed, merged app and patch hex file ${CY_APP_SIGNED_HEX}.ss_ds.hex was not created"
    exit 1
fi

APPEND_PATCH_APP_HEX_COMMAND="${CYWICEDTOOLS}/IntelHexToBin/AppendToIntelHex NEWPARAMS "
APPEND_PATCH_APP_HEX_COMMAND+="-EXTRADATAFILE ${CY_APP_HEX}.ds.bin "
APPEND_PATCH_APP_HEX_COMMAND+="-EXTRADATAADDRESS ${patch_ds_end} "
APPEND_PATCH_APP_HEX_COMMAND+="-INHEXFILE ${CY_APP_SIGNED_HEX}.ss_ds.hex "
APPEND_PATCH_APP_HEX_COMMAND+="-OUTPUTINTELHEXBYTESPERLINE 240 "
if [[ $CY_APP_BUILD_EXTRAS = *"DIRECT"* ]]; then
    CY_APP_HEX_APPENDED="${CY_APP_HEX}.ss_ds_app.hex"
else
    CY_APP_HEX_APPENDED="${CY_APP_HEX}"
fi
APPEND_PATCH_APP_HEX_COMMAND+="-OUTHEXFILE ${CY_APP_HEX_APPENDED}"
if [ ${VERBOSE} -ne 0 ]; then
    echo Calling ${APPEND_PATCH_APP_HEX_COMMAND}
fi
set +e
eval ${APPEND_PATCH_APP_HEX_COMMAND}
set -e
# check that this completed
if [[ ! -e ${CY_APP_HEX_APPENDED} ]]; then
    echo "!! Post build failed, file ${CY_APP_HEX_APPENDED} was not created"
    exit 1
fi

# for DIRECT_LOAD, append DL hex
if [[ $CY_APP_BUILD_EXTRAS = *"DIRECT"* ]]; then
    CONCAT_ALL_HEX_COMMAND="${CYWICEDTOOLS}/IntelHexToBin/ConcatIntelHex "
    CONCAT_ALL_HEX_COMMAND+="${CY_APP_HEX}.ss_ds_app.hex ${CY_APP_HEX}.dl.hex ${CY_APP_SIGNED_HEX}.dl.hex ${CY_APP_HEX}"
    if [ ${VERBOSE} -ne 0 ]; then
        echo Calling ${CONCAT_ALL_HEX_COMMAND}
    fi
    set +e
    eval ${CONCAT_ALL_HEX_COMMAND}
    set -e

    if [[ -e $CY_APP_CGS_MAP ]]; then
        # get last RAM addr for DIRECT_LOAD
        CY_APP_CGS_MAP_LAST_LINE=$($CY_TOOL_TAIL -1 "$CY_APP_CGS_MAP")
        CY_APP_END_DS=$(echo $CY_APP_CGS_MAP_LAST_LINE | "$CY_TOOL_PERL" -ple 's/[^0]+(0x[0-9A-Fa-f]+)/$1/')
        # get last SRAM addr
        CY_APP_END_SRAM=$("$CY_TOOL_PERL" -ne 'print "$1" if /end SRAM (0x[0-9A-F]+)/' "$CY_RESOURCE_REPORT")
        if [[ "$CY_APP_END_DS" -gt "$CY_APP_END_SRAM" ]]; then
            echo "!! Error: DIRECT_LOAD SS/DS end ($CY_APP_END_DS) would exceed SRAM end ($CY_APP_END_SRAM)"
            exit 1
        fi
    fi
fi

"${CYWICEDTOOLS}/IntelHexToBin/IntelHexToHCD" "$CY_APP_HEX" "$CY_APP_HCD"

# copy files necessary for download to help launch config find them
"$CY_TOOL_CP" "$CY_APP_BTP" "$CY_MAINAPP_BUILD_DIR/$CY_MAINAPP_NAME.btp"
"$CY_TOOL_CP" "$CY_APP_HCI_ID" "$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}_hci_id.txt"
if [[ -e $CY_APP_BAUDRATE_FILE ]]; then
"$CY_TOOL_CP" "$CY_APP_BAUDRATE_FILE" "$CY_MAINAPP_BUILD_DIR/${CY_MAINAPP_NAME}_baudrates.txt"
fi
"$CY_TOOL_CP" "$CY_APP_MINIDRIVER" "$CY_MAINAPP_BUILD_DIR/minidriver.hex"
"$CY_TOOL_ECHO" "$CY_APP_CHIPLOAD_FLAGS" >"$CY_MAINAPP_BUILD_DIR/chipload_flags.txt"

echo "Post build processing completed"
