######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll

# The paths that cmake substitutues in the code_generator_t constructor might
# contain Unicode characters.
# coding: utf-8

import logging
import time
import threading
import sys
from pygccxml import declarations, parser
from pyplusplus import module_builder, messages
from pyplusplus.module_builder import call_policies
from pyplusplus.decl_wrappers import print_declarations

# disable some warnings that are mostly harmless
messages.disable( messages.W1014 )
messages.disable( messages.W1023 )
messages.disable( messages.W1040 )

# create a dictionary called "default_replacement" that can be used to replace method
# declaration with another method. The dictionary is indexed by method name,
# either with or without full scope. For each method there is a 2-tuple. The
# first component is a string that tells Boost.Python how to export the
# replacement method. The second component is a string that contains the C++
# code for the replacement method.
default_replacement = {}
# A C++ call like "foo.print(std::cout)" will be replaced with something more
# pythonesque using the special __str__ method: "print(foo)"
default_replacement['print'] = ('def("__str__", &__str__)', """
std::string __str__(%s* obj)
{
    std::ostringstream s;
    obj->print(s);
    return s.str();
}
""")
# A C++ call like "foo.printSettings(std::cout)" will be replaced with
# something more pythonesque: "print(foo.settings())"
default_replacement['printSettings'] = ('def("settings", &__printSettings)', """
std::string __printSettings(%s* obj)
{
    std::ostringstream s;
    obj->printSettings(s);
    return s.str();
}
""")

# add array read access to objects that can return a const reference using the
# [] operator.
default_replacement['array_read_access'] = ('def("__getitem__", &__getitem)',"""
%s __getitem(%s* obj, unsigned int i)
{
    return (*obj)[i];
}
""")
# add array write access to objects that can return a non-const reference
# using the [] operator.
default_replacement['array_write_access'] = ('def("__setitem__", &__setitem)',"""
void __setitem(%s* obj, unsigned int i, %s val)
{
    (*obj)[i] = val;
}
""")

# impl_conv_code = """
# boost::python::implicitly_convertible< std::shared_ptr< %(from)s >, std::shared_ptr< %(to)s > >();
# """

def returns_reference(decl):
    """Return True iff the declaration returns a pointer or reference."""
    c = decl.return_type.decl_string[-1]
    return c=='&' or c=='*'

class Heartbeat(object):
    def __init__(self, delay):
        self.start = time.time()
        self.delay = delay
    def __call__(self):
        while True:
            time.sleep(self.delay)
            t = time.time()
            print('still alive after %g seconds!' % (t - self.start))
            sys.stdout.flush()

class code_generator_t(object):
    """The base class code generator."""

    def __init__(self, name, deps=None, replacement_dict=default_replacement, indexing_suite_version = 2):
        """Constructor.
        @name name of the python module
        @dep name of another module this module depends on"""
        # create a separate thread that prints out a message every 60 seconds
        # to let build bots know the build is not stuck.
        self.heartbeat = threading.Thread(target = Heartbeat(60))
        self.heartbeat.daemon = True
        self.heartbeat.start()
        xml_generator_config = parser.load_xml_generator_configuration("/tmp/QtCreator-m9KY4a/qtc-cmake-XXsdnJC8/castxml.cfg")
        self.mb = module_builder.module_builder_t(
            files = [ 'bindings/' + name + '.h' ],
            # cache is not used with compilation_mode = parser.COMPILATION_MODE.ALL_AT_ONCE
            #cache = '/tmp/QtCreator-m9KY4a/qtc-cmake-XXsdnJC8/pyplusplus_'+name+'.cache',
            xml_generator_config = xml_generator_config,
            start_with_declarations = ['ompl::' + name],
            compilation_mode = parser.COMPILATION_MODE.ALL_AT_ONCE,
            indexing_suite_version = indexing_suite_version )
        self.replacement = {} if replacement_dict==None else replacement_dict
        self.mb.classes().always_expose_using_scope = True
        self.mb.add_registration_code('PyEval_InitThreads();', tail=False)
        self.std_ns = self.mb.namespace('std')
        self.ompl_ns = self.mb.namespace('ompl')
        self.call_policies()
        self.filter_declarations()
        if deps!=None:
            for dep in deps:
                self.mb.register_module_dependency(dep)
        self.mb.build_code_creator( module_name='_'+name )
        self.mb.split_module('bindings/' + name, use_files_sum_repository=True)

    def call_policies(self):
        """Define default call policies: for anything that returns a reference,
        return a reference to the existing object. This is potentially dangerous."""
        decllist = [ self.mb.member_functions(returns_reference),
            self.mb.free_functions(returns_reference),
            self.mb.operators(returns_reference) ]
        for decls in decllist:
            for decl in decls:
                if decl.return_type.decl_string[-7:]=='const &':
                    decl.call_policies = \
                        call_policies.return_value_policy(
                            call_policies.copy_const_reference)
                else:
                    decl.call_policies = \
                        call_policies.return_value_policy(
                            call_policies.reference_existing_object)

    def filter_declarations(self):
        """Remove some classes and functions from the std namespace"""
        try:
            self.std_ns.class_('ios_base').exclude()
        except:
            pass
        self.std_ns.free_functions().exclude()
        self.std_ns.operators().exclude()

    def replace_member_function(self, decl):
        """Utility function to replace a declaration."""
        decl.exclude()
        cls = decl.parent

        self.mb.logger.info('Replacing member function ' + cls.decl_string+'::'+decl.name)
        try:
            (reg, wrapper) = self.replacement[cls.decl_string+'::'+decl.name]
        except KeyError:
            (reg, wrapper) = self.replacement[decl.name]
        cls.add_registration_code(reg)
        cls.add_declaration_code(wrapper % cls.decl_string)

    def replace_member_functions(self, decls):
        """Utility function to replace multiple declarations."""
        for decl in decls:
             self.replace_member_function(decl)

    def add_array_access(self, cls, rettype=None):
        """Add array indexing to a class."""
        op = cls.operators("[]")
        op.exclude()
        self.mb.logger.info('Adding array access to class ' + cls.decl_string)
        (reg, wrapper) = self.replacement['array_read_access']
        cls.add_registration_code(reg)
        if not rettype: rettype=op[0].return_type.decl_string
        cls.add_declaration_code(wrapper % (rettype, cls.decl_string))
        (reg, wrapper) = self.replacement['array_write_access']
        cls.add_registration_code(reg)
        cls.add_declaration_code(wrapper % (cls.decl_string, rettype))

    def add_function_wrapper(self, FT, func_name, func_doc):
        self.mb.add_declaration_code('PYDECLARE_FUNCTION(%s,%s)' % (FT, func_name))
        self.mb.add_registration_code('PYREGISTER_FUNCTION(%s,%s,"%s")' % (FT, func_name, func_doc))

    # Boost.Python has special code to handle boost::shared_ptr conversions,
    # but support for std::shared_ptr is still missing (see
    # https://github.com/boostorg/python/issues/29). This function attempts
    # to (partially) fix this for specific classes.

    # We currently use a different workaround: copy the special bits of Boost
    # code that handle boost::shared_ptr's and modify them to use
    # std::shared_ptr's instead. Might not be portable across a wide range of
    # Boost.Python versions....

    # def add_implicit_conversions(self, cls):
    #     cls.held_type = 'std::shared_ptr< %s >' % cls.wrapper_alias
    #     cls.add_registration_code(impl_conv_code % {'from': cls.wrapper_alias, 'to': cls.decl_string}, False)
    #     for base in cls.recursive_bases:
    #         if base.access_type == 'public':
    #             # from class to its base
    #             cls.add_registration_code(impl_conv_code % {'from': cls.decl_string, 'to': base.related_class.decl_string}, False)
    #             # from wrapper to clas base class
    #             cls.add_registration_code(impl_conv_code % {'from': cls.wrapper_alias, 'to': base.related_class.decl_string}, False)
