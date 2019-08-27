#!/usr/bin/python3

# Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program.  If not, see
# <http://www.gnu.org/licenses/>.

import cmd, sys
from types import MethodType
import subprocess

class BrushlessShell(cmd.Cmd):
    intro = 'Welcome to the brushless shell.   Type help or ? to list commands.\n'
    prompt = '$ '

    def __init__(self):
        cmd.Cmd.__init__(self)
        try:
            res = subprocess.check_output(["bash", "./extract_command.sh"])
        except:
            print("Unexpected error:", sys.exc_info()[0])
            raise
        import terminal_command as all_commands

        def insert_command(name, doc, function, doc_function):
            if name == 'help':
                name = 'help_brushless'
            setattr(
                self.__class__, 'do_'+name,
                MethodType( lambda self,arg: function(arg), self )
            )
            setattr(
                self.__class__, 'help_'+name,
                MethodType( doc_function, self )
            )
        for (name,doc) in all_commands.terminal_command:
            function = eval( "lambda arg : print('TODO %s : ' + str(arg))"%(name))
            help_fct = eval( "lambda arg : print('%s <command>')"%(doc))
            insert_command(name, doc, function, help_fct)
        for (name,doc) in all_commands.parameter_int:
            function = eval( "lambda arg : print('TODO %s : ' + str(arg))"%(name))
            help_fct = eval( "lambda arg : print('%s <integer parameter>')"%(doc))
            insert_command(name, doc, function, help_fct)
        for (name,doc) in all_commands.parameter_float:
            function = eval( "lambda arg : print('TODO %s : ' + str(arg))"%(name))
            help_fct = eval( "lambda arg : print('%s <float parameter>')"%(doc))
            insert_command(name, doc, function, help_fct)
        for (name,doc) in all_commands.parameter_bool:
            function = eval( "lambda arg : print('TODO %s : ' + str(arg))"%(name))
            help_fct = eval( "lambda arg : print('%s <boolean parameter>')"%(doc))
            insert_command(name, doc, function, help_fct)
    def do_quit(self, arg):
        'Quit the command shell.'
        return True
    def do_exit(self, arg):
        'Same as quit command.'
        return True

def parse(arg):
    'Convert a series of zero or more numbers to an argument tuple'
    return list(arg.split())

if __name__ == '__main__':
    BrushlessShell().cmdloop()
