import os

def get_unique_name(base_name, is_directory=False, extension=None):
    """
    Generate a unique name by appending a number if necessary.
    Args:
        base_name (str): The base name of the directory or file.
        is_directory (bool): Whether the name is for a directory.
        extension (str): File extension if it's a file (e.g., ".png").
    Returns:
        str: A unique name for the directory or file.
    """
    counter = 1
    if is_directory:
        unique_name = base_name
        while os.path.exists(unique_name):
            unique_name = f"{base_name}_{counter}"
            counter += 1
        return unique_name
    else:
        if extension is None:
            raise ValueError("File extension must be specified for files.")
        unique_name = f"{base_name}{extension}"
        while os.path.exists(unique_name):
            unique_name = f"{base_name}_{counter}{extension}"
            counter += 1
        return unique_name