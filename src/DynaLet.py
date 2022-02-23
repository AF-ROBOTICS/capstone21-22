from statistics import mean
import PathBuild

MAX_BOTS = 25
letter_library = {
    'A': ([0.0, 0.6, 0.15, 0.45, 0.3, 0.3],
          [0.0, 0.0, 0.50, 0.50, 1.0, 0.5]),
    # TODO: B looks kinda weird
    'B': ([0.0, 0.0, 0.0, 0.60, 0.60, 0.40, 0.4, 0.4],
          [0.0, 0.5, 1.0, 0.25, 0.75, 0.50, 0.0, 1.0]),

    'C': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 1.0]),

    'D': ([0.0, 0.0, 0.0, 0.30, 0.30, 0.60],
          [0.0, 0.5, 1.0, 0.25, 0.75, 0.50]),

    'E': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.0, 1.0]),

    'F': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6],
          [0.0, 0.5, 1.0, 0.5, 1.0, 1.0]),

    'G': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6, 0.3],
          [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 1.0, 0.3, 0.3]),

    'H': ([0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.5, 0.0, 0.5, 1.0]),

    'I': ([0.0, 0.3, 0.6, 0.3, 0.0, 0.3, 0.6],
          [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]),

    'J': ([0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.0],
          [0.0, 0.3, 0.0, 0.5, 1.0, 1.0, 1.0]),

    'K': ([0.0, 0.0, 0.0, 0.30, 0.30, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.25, 0.75, 0.0, 1.0]),

    'L': ([0.0, 0.0, 0.0, 0.3, 0.6],
          [0.0, 0.5, 1.0, 0.0, 0.0]),

    'M': ([0.0, 0.0, 0.0, 0.15, 0.3, 0.45, 0.6, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.75, 0.5, 0.75, 1.0, 0.5, 0.0]),

    'N': ([0.0, 0.0, 0.0, 0.15, 0.3, 0.45, 0.6, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.75, 0.5, 0.25, 0.0, 0.5, 1.0]),

    'O': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.5, 0.0]),

    'P': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 1.0]),

    'Q': ([0.00, 0.0, 0.0, 0.20, 0.2, 0.4, 0.40, 0.4, 0.6],
          [0.25, 0.7, 1.0, 0.25, 1.0, 1.0, 0.25, 0.7, 1.0]),
    # TODO: extra bots?
    'R': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6, 0.45],
          [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 1.0, 0.0, 0.25]),

    'S': ([0.0, 0.0, 0.0, 0.00, 0.3, 0.3, 0.3, 0.6, 0.6, 0.6, 0.60],
          [0.0, 0.5, 1.0, 0.75, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.25]),

    'T': ([0.0, 0.3, 0.3, 0.3, 0.6],
          [1.0, 0.0, 0.5, 1.0, 1.0]),

    'U': ([0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.0, 0.0, 0.5, 1.0]),

    'V': ([0.0, 0.15, 0.3, 0.45, 0.6],
          [1.0, 0.50, 0.0, 0.50, 1.0]),

    'W': ([0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.5, 0.0, 0.5, 1.0]),

    'X': ([0.0, 0.0, 0.3, 0.6, 0.6],
          [0.0, 1.0, 0.5, 0.0, 1.0]),

    'Y': ([0.0, 0.3, 0.3, 0.6],
          [1.0, 0.5, 0.0, 1.0]),

    'Z': ([0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.6, 0.15, 0.45],
          [0.0, 1.0, 0.0, 0.5, 1.0, 0.0, 1.0, 0.75, 0.25])
}


def split(string):
    string = string.upper()
    return [char for char in string]


def custom_word():
    x_destinations = []
    y_destinations = []
    word_sum = 0
    # Get user input. Only letters in dictionary allowed and cannot use more bots than there are
    while not 0 < word_sum <= MAX_BOTS:
        word_sum = 0
        usr_word = input("Enter word for robots to spell: ")
        word_parsed = split(usr_word)
        for letter in word_parsed:
            word_sum = word_sum + len(letter_library.get(letter, [[], []])[0])

    # Separate each letter by 1 meter (box)
    for position, letter in enumerate(word_parsed):
        x_cords = letter_library.get(letter, [[], []])[0]
        y_cords = letter_library.get(letter, [[], []])[1]
        for x_cord in x_cords:
            x_destinations.append(x_cord + position)
        for y_cord in y_cords:
            y_destinations.append(y_cord + 2)

    # Shift phrase to center of map
    center_offset = 2.5 - mean(x_destinations)
    for each in range(len(x_destinations)):
        x_destinations[each] = x_destinations[each] + center_offset

    return x_destinations, y_destinations, usr_word


if __name__ == '__main__':
    while True:
        x_dyna, y_dyna, word = custom_word()
        if PathBuild.check_cache(word):
            x_dyna, y_dyna = PathBuild.check_cache(word)
        start_points, end_points = PathBuild.pack_to_points(x_dyna, y_dyna)
        x, y = PathBuild.build_path(start_points, end_points)
        PathBuild.add_to_cache(word, x, y)
