exxx_table_path() {
    home_path=~/.control_table/control_table.csv
    if [ -f $home_path ] ; then
        control_table_path=$home_path
    else
        control_table_path=$(\ls -rv $(ros2 pkg prefix exxx_control_table)/share/exxx_control_table/control_tables/v{?,??}.{?,??}.{?,??}/control_table.csv 2>/dev/null | xargs | cut -d' ' -f1)
    fi
    echo $control_table_path
}

comp_exxx_read () {
    case "$COMP_CWORD" in
    1)
        COMPREPLY=( `compgen -W "$(ls /dev/tty*)"  $2` );;
    2)
        COMPREPLY=( `compgen -W "$(seq -s ' ' 99)" $2` );;
    3)
        COMPREPLY=( `compgen -W "$(cut -d ',' -f 3 $(exxx_table_path))" $2` );;
    4)
        COMPREPLY=( `compgen -W "--usb" $2` );;
    esac
}
comp_exxx_write () {
    case "$COMP_CWORD" in
    1)
        COMPREPLY=( `compgen -W "$(ls /dev/tty*)"  $2` );;
    2)
        COMPREPLY=( `compgen -W "$(seq -s ' ' 99)" $2` );;
    3)
        COMPREPLY=( `compgen -W "$(cut -d ',' -f 3 $(exxx_table_path))" $2` );;
    5)
        COMPREPLY=( `compgen -W "--usb" $2` );;
    esac
}
comp_exxx_hash () {
    case "$COMP_CWORD" in
    1)
        COMPREPLY=( `compgen -W "$(ls /dev/tty*)"  $2` );;
    2)
        COMPREPLY=( `compgen -W "$(seq -s ' ' 99)" $2` );;
    3)
        COMPREPLY=( `compgen -W "--usb" $2` );;
    esac
}
comp_exxx_eeprom () {
    case "$COMP_CWORD" in
    1)
        COMPREPLY=( `compgen -W "$(ls /dev/tty*)"  $2` );;
    2)
        COMPREPLY=( `compgen -W "$(seq -s ' ' 99)" $2` );;
    3)
        COMPREPLY=( `compgen -W "--usb" $2` );;
    esac
}
comp_exxx_flush () {
    case "$COMP_CWORD" in
    1)
        COMPREPLY=( `compgen -W "$(ls /dev/tty*)"  $2` );;
    2)
        COMPREPLY=( `compgen -W "11 12 13 21 22 23 24 25 31 32 41" $2` );;
    3)
        compopt -o filenames
        COMPREPLY=( `compgen -f -X "!*.mot" $2` );;
    4)
        COMPREPLY=( `compgen -W "--usb --compulsion" $2` );;
    5)
        if [ ${COMP_WORDS[COMP_CWORD-1]} == '--usb' ] ; then
            COMPREPLY=( `compgen -W "--compulsion" $2` )
        elif [ ${COMP_WORDS[COMP_CWORD-1]} == '--compulsion' ] ; then
            COMPREPLY=( `compgen -W "--usb" $2` )
        else
            COMPREPLY=( `compgen -W "--usb --compulsion" $2` )
        fi
    esac
}


alias exxx_read='ros2 run hsrb_servomotor_protocol exxx_read_data_table'
alias exxx_write='ros2 run hsrb_servomotor_protocol exxx_write_data_table'
alias exxx_hash='ros2 run hsrb_servomotor_protocol exxx_read_hash'
alias exxx_eeprom='ros2 run hsrb_servomotor_protocol exxx_write_eeprom'
alias exxx_flush='ros2 run hsrb_servomotor_protocol exxx_flush'
complete -F comp_exxx_read exxx_read
complete -F comp_exxx_write exxx_write
complete -F comp_exxx_hash exxx_hash
complete -F comp_exxx_eeprom exxx_eeprom
complete -F comp_exxx_flush exxx_flush
