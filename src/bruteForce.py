#!/usr/bin/env python3
# +----------------------------------------------------------------------------
# |
# | United States Air Force Academy     __  _______ ___    _________
# | Dept of Electrical &               / / / / ___//   |  / ____/   |
# | Computer Engineering              / / / /\__ \/ /| | / /_  / /| |
# | 2354 Fairchild Drive Ste 2F6     / /_/ /___/ / ___ |/ __/ / ___ |
# | USAF Academy, CO 80840           \____//____/_/  |_/_/   /_/  |_|
# |
# | ---------------------------------------------------------------------------
# |
# | FILENAME      : bruteForce.py
# | AUTHOR(S)     : C1C Anthony Tolbert
# | CREATED       : 29 Mar 2022
# | Last Update   : 18 Apr 2022
"""
This is a test module that runs through every possible phrase that could be entered by the user and attempts to build a
 waterfall path with that input

This module was orginally designed to run over Spring Break '22 (approx. 10 days) as the only process running. Upon
return, the program had made it through 'AAF'. Needless to say this is a time intensive process.

This script requires:
    * DynaLet
    * PathBuild

This file contains 1 standalone function and a main function:
    Functions
    ---------
    * make_list : Create list of strings from "A" to "ZZZZZ"
"""
import PathBuild
import DynaLet


# iterate over uppercase ascii range [65,190] for 5 letters
def make_list():
    """Create list of strings from "A" to "ZZZZZ" """
    words = []
    j = k = l = m = -1
    for i in range(12356630):  # not the true number of iterations
        word = []
        if i % 26 == 0 and i != 0:
            j += 1
            if j % 26 == 0 and j != 0:
                k += 1
                if k % 26 == 0 and k != 0:
                    l += 1
                    if l % 26 == 0 and l != 0:
                        m += 1
        word += chr(i % 26 + 65)
        if j >= 0:
            word += chr(j % 26 + 65)
        if k >= 0:
            word += chr(k % 26 + 65)
        if l >= 0:
            word += chr(l % 26 + 65)
        if m >= 0:
            word += chr(m % 26 + 65)
        word.reverse()
    return words


if __name__ == '__main__':
    words = make_list()
    print('made words')
    for word in words:
        x, y, wrd = DynaLet.custom_word(word)
        starts, ends = PathBuild.pack_to_points(x, y)
        x_pts, y_pts = PathBuild.build_path(starts, ends)
        if len(x_pts):  # only do this if it worked
            PathBuild.add_to_cache(word.upper(), x_pts, y_pts)
            print(f"added \'{word}\' to cache")
